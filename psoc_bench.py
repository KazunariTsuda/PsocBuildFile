#!/usr/bin/env python3
"""Bench counterpart for the PSoC CAN changes.

Sends the drive frame that sets bRaspInit (so 0x092 starts) at speed 0, and
prints what the PSoC sends back. Does not bring can0 down when it is already up.

Drive bytes match motor_zero_can_tx.py: the PSoC RX mailbox presents each
32-bit word reversed, so wire data[0] is the device byte the ISR reads as byte[3].

  sudo python3 psoc_bench.py
  sudo python3 psoc_bench.py --no-tx
  sudo python3 psoc_bench.py --seconds 15
  sudo python3 psoc_bench.py --tx-id 0x080

See README_BENCH.md for the Pi address and NTP server.
"""

from __future__ import annotations

import argparse
import errno
import re
import select
import signal
import socket
import struct
import subprocess
import sys
import time
from datetime import datetime, timezone

AF_CAN = getattr(socket, "AF_CAN", 29)
CAN_RAW = getattr(socket, "CAN_RAW", 1)
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_ERR_MASK = 0x1FFFFFFF
SOL_CAN_RAW = 101
CAN_RAW_FILTER = 1
CAN_RAW_ERR_FILTER = 2
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

ID_SYNC = 0x080
ID_TEMP = 0x090
ID_HEALTH = 0x092
ID_SWITCH = 0x0A0
ID_MOTOR1 = 0x0B0
ID_MOTOR2 = 0x0B1
ID_DRIVE_BENCH = 0x101

HEALTH_FLAGS = (
    (0x01, "stopped"),
    (0x02, "gave_up"),
    (0x04, "tx_warn"),
    (0x08, "rx_warn"),
    (0x10, "passive"),
    (0x20, "bus_off"),
)

RE_STATE = re.compile(r"can state (\S+)")
RE_BERR = re.compile(r"berr-counter tx (\d+) rx (\d+)")
RE_CLOCK = re.compile(r"clock (\d+)")
RE_BUSOFF = re.compile(
    r"re-started\s+bus-errors\s+arbit-lost\s+error-warn\s+error-pass\s+bus-off\s+"
    r"(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)",
    re.S,
)

stop_requested = False


def on_signal(_signum: int, _frame: object) -> None:
    global stop_requested
    stop_requested = True


def log(text: str) -> None:
    stamp = datetime.now(timezone.utc).strftime("%H:%M:%S")
    print(f"{stamp}Z {text}", flush=True)


def parse_id(text: str) -> int:
    value = int(text, 0)
    if value < 0 or value > 0x7FF:
        raise argparse.ArgumentTypeError(f"CAN ID out of range: {text}")
    return value


def link_text(iface: str) -> str:
    try:
        proc = subprocess.run(
            ["ip", "-details", "-statistics", "link", "show", iface],
            check=False,
            capture_output=True,
            text=True,
        )
    except OSError as exc:
        return f"(ip failed: {exc})"
    return (proc.stdout or "") + (proc.stderr or "")


def iface_is_up(text: str) -> bool:
    first = text.split("\n", 1)[0]
    return " state UP " in first


def bring_up(iface: str) -> None:
    text = link_text(iface)
    if "does not exist" in text or text.startswith("(ip failed"):
        raise SystemExit(
            f"{iface} がありません。先に rpi_mcp2515_setup/README.md の手順で overlay を入れて再起動してください。"
        )
    if iface_is_up(text):
        log(f"{iface} は既に UP。down しません。")
        return
    clock = RE_CLOCK.search(text)
    clk = int(clock.group(1)) if clock else 0
    ip = "ip"
    if clk == 8000000:
        cmd = [
            ip, "link", "set", iface, "type", "can",
            "tq", "125", "prop-seg", "6", "phase-seg1", "7", "phase-seg2", "2",
            "sjw", "2", "restart-ms", "100",
        ]
        note = "16 MHz 水晶 (clock 8000000, sample 0.875, sjw 2)"
    elif clk == 4000000:
        cmd = [
            ip, "link", "set", iface, "type", "can",
            "bitrate", "500000", "sample-point", "0.750", "sjw", "1",
            "restart-ms", "100",
        ]
        note = "8 MHz 水晶 (clock 4000000, sample 0.750)"
    else:
        raise SystemExit(
            f"{iface} の clock が読めません ({clk})。rpi_mcp2515_setup の check を先に実行してください。"
        )
    log(f"{iface} を上げます。{note}")
    subprocess.run(cmd, check=True)
    subprocess.run([ip, "link", "set", iface, "txqueuelen", "1000"], check=True)
    subprocess.run([ip, "link", "set", iface, "up"], check=True)


def pack_drive(can_id: int, dev: int) -> bytes:
    """Speed 0, bAct 0. Wire order matches motor_zero_can_tx.pack_motor."""
    data = bytes([dev & 0xFF, 0, 0, 0, 0, 0, 0, 0])
    return struct.pack(CAN_FRAME_FMT, can_id & CAN_SFF_MASK, 8, data)


def open_can(iface: str) -> socket.socket:
    sock = socket.socket(AF_CAN, socket.SOCK_RAW, CAN_RAW)
    try:
        sock.bind((iface,))
    except OSError as exc:
        sock.close()
        raise SystemExit(f"{iface} を開けません ({exc})。インターフェースが UP か、sudo か確認してください。") from exc
    err_mask = struct.pack("=I", CAN_ERR_MASK)
    try:
        sock.setsockopt(SOL_CAN_RAW, CAN_RAW_ERR_FILTER, err_mask)
    except OSError:
        sock.setsockopt(100, CAN_RAW_ERR_FILTER, err_mask)
    filt = struct.pack("=II", 0, 0)
    try:
        sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER, filt)
    except OSError:
        pass
    sock.setblocking(False)
    return sock


def u16be(data: bytes, offset: int) -> int:
    return (data[offset] << 8) | data[offset + 1]


def u32be(data: bytes, offset: int) -> int:
    return (
        (data[offset] << 24)
        | (data[offset + 1] << 16)
        | (data[offset + 2] << 8)
        | data[offset + 3]
    )


def flag_text(flags: int) -> str:
    names = [name for bit, name in HEALTH_FLAGS if flags & bit]
    if not names:
        return "none"
    return ",".join(names)


def describe(can_id: int, dlc: int, data: bytes) -> str | None:
    if can_id == ID_HEALTH and dlc >= 6:
        count = (data[2] << 8) | data[3]
        errst = data[6] if dlc >= 7 else 0
        return (
            f"HEALTH tec={data[0]} rec={data[1]} busoff_count={count} "
            f"attempts={data[4]} flags={flag_text(data[5])} errst=0x{errst:02x}"
        )
    if can_id == ID_TEMP and dlc >= 8:
        return (
            f"TEMP amp={u16be(data, 0)} t1={u16be(data, 2)} "
            f"t2={u16be(data, 4)} t3={u16be(data, 6)}"
        )
    if can_id == ID_SWITCH and dlc >= 4:
        return f"SWITCH pwr={data[0]} remote={data[1]} safety={data[2]} ems={data[3]}"
    if can_id in (ID_MOTOR1, ID_MOTOR2) and dlc >= 6:
        node = 1 if can_id == ID_MOTOR1 else 2
        sts = u16be(data, 0)
        din = u32be(data, 2)
        op_en = (sts >> 2) & 1
        return f"DRIVER{node} status=0x{sts:04x} op_en={op_en} din=0x{din:08x}"
    if can_id == ID_SYNC and dlc == 0:
        return None
    return None


def describe_error(can_id: int, data: bytes) -> str:
    kind = []
    if can_id & 0x40:
        kind.append("bus-off")
    if can_id & 0x04:
        kind.append("ctrl")
    if can_id & 0x20:
        kind.append("ack")
    if can_id & 0x100:
        kind.append("restarted")
    if can_id & 0x80:
        kind.append("bus-error")
    label = ",".join(kind) if kind else "error"
    return f"ERR {label} id=0x{can_id & CAN_ERR_MASK:08x} data={data[:8].hex()}"


def link_summary(iface: str) -> str:
    text = link_text(iface)
    state = RE_STATE.search(text)
    berr = RE_BERR.search(text)
    bus = RE_BUSOFF.search(text)
    parts = [state.group(1) if state else "state?"]
    if berr:
        parts.append(f"tec={berr.group(1)} rec={berr.group(2)}")
    if bus:
        parts.append(f"bus_off={bus.group(6)} restarts={bus.group(1)}")
    return " ".join(parts)


def send_frame(can: socket.socket, raw: bytes) -> bool:
    try:
        can.send(raw)
        return True
    except (BlockingIOError, OSError) as exc:
        err = getattr(exc, "errno", None)
        if err in (errno.EAGAIN, errno.EWOULDBLOCK, errno.ENOBUFS):
            return False
        log(f"TX 失敗: {exc}")
        return False


def main() -> int:
    parser = argparse.ArgumentParser(description="PSoC 対向の動作確認")
    parser.add_argument("--iface", default="can0")
    parser.add_argument(
        "--tx-id",
        type=parse_id,
        default=ID_DRIVE_BENCH,
        help="駆動フレームの ID。既定は 0x101。本番の 0x080 は SYNC と重なる",
    )
    parser.add_argument("--period-ms", type=int, default=200)
    parser.add_argument("--no-tx", action="store_true", help="受信だけ。0x092 は bRaspInit まで出ない")
    parser.add_argument("--bring-up", action="store_true", help="can0 が DOWN のときだけ上げる")
    parser.add_argument("--seconds", type=float, default=0.0, help="0 なら Ctrl+C まで")
    args = parser.parse_args()
    if args.period_ms < 20:
        parser.error("--period-ms は 20 以上")

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    if args.bring_up:
        bring_up(args.iface)

    can = open_can(args.iface)
    log(f"listen {args.iface}  link: {link_summary(args.iface)}")
    if args.no_tx:
        log("送信なし。PSoC は駆動フレームを受けるまで 0x092 を出しません。")
    else:
        log(
            f"駆動を {args.period_ms} ms ごとに {args.tx_id:#05x} へ送ります。"
            " 速度 0、bAct 0。PSoC はこれを受けるとモータへ速度 0 を出し続けます。"
        )

    left = pack_drive(args.tx_id, 1)
    right = pack_drive(args.tx_id, 2)
    next_tx = time.monotonic()
    next_link = time.monotonic() + 5.0
    started = time.monotonic()
    last_health = 0.0
    health_count = 0
    sync_count = 0
    tx_fail = 0
    silent_noted = False

    while not stop_requested:
        if args.seconds and (time.monotonic() - started) >= args.seconds:
            break
        now = time.monotonic()
        timeout = 0.2
        if not args.no_tx:
            timeout = max(0.0, min(timeout, next_tx - now))
        readable, _, _ = select.select([can], [], [], timeout)
        if readable:
            while True:
                try:
                    raw = can.recv(CAN_FRAME_SIZE)
                except BlockingIOError:
                    break
                except OSError as exc:
                    log(f"RX 失敗: {exc}")
                    break
                if len(raw) < CAN_FRAME_SIZE:
                    break
                can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw[:CAN_FRAME_SIZE])
                if can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG):
                    continue
                if can_id & CAN_ERR_FLAG:
                    log(describe_error(can_id, data))
                    continue
                sid = can_id & CAN_SFF_MASK
                if sid == ID_SYNC and dlc == 0:
                    sync_count += 1
                    continue
                if sid == args.tx_id and dlc == 8:
                    continue
                text = describe(sid, dlc, data[:dlc])
                if sid == ID_HEALTH:
                    health_count += 1
                    last_health = time.monotonic()
                    silent_noted = False
                if text:
                    log(text)
                else:
                    log(f"RX {sid:#05x} dlc={dlc} {data[:dlc].hex()}")

        now = time.monotonic()
        if not args.no_tx and now >= next_tx:
            ok_l = send_frame(can, left)
            ok_r = send_frame(can, right)
            if not (ok_l and ok_r):
                tx_fail += 1
                if tx_fail == 1 or tx_fail % 20 == 0:
                    log(f"駆動 TX がキューに載りません (累計 {tx_fail})")
            next_tx = now + (args.period_ms / 1000.0)

        if last_health and (now - last_health) > 3.0 and not silent_noted:
            log("0x092 が 3 秒来ていません。PSoC の CAN が止まっているか、bRaspInit が落ちています。")
            silent_noted = True

        if now >= next_link:
            log(
                f"link {link_summary(args.iface)}  "
                f"health={health_count} sync={sync_count}"
            )
            next_link = now + 5.0

    can.close()
    elapsed = time.monotonic() - started
    log(
        f"終了 {elapsed:.0f}s  health={health_count} sync={sync_count} tx_fail={tx_fail}"
    )
    if health_count == 0 and not args.no_tx:
        log("0x092 を一度も受信していません。CAN の結線、終端、PSoC の起動、駆動 ID を確認してください。")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
