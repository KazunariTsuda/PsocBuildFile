#!/usr/bin/env python3
"""走行中の TPDO1（0x181 / 0x182）途絶を監視する。

左右それぞれについて、前回受信からの経過が --gap 秒を超えたら検知する。
片方だけ止まる片側途絶も同じ閾値で出す。走行アプリとは別プロセスで動かす。

  python3 watch_torque_pdo.py
  python3 watch_torque_pdo.py --iface can0 --gap 1.0
  python3 watch_torque_pdo.py --gap 3.0 --log torque_gaps.csv
"""

from __future__ import annotations

import argparse
import csv
import select
import signal
import socket
import struct
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

AF_CAN = getattr(socket, "AF_CAN", 29)
CAN_RAW = getattr(socket, "CAN_RAW", 1)
SOL_CAN_RAW = 101
CAN_RAW_FILTER = 1
CAN_SFF_MASK = 0x000007FF
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

ID_LEFT = 0x181
ID_RIGHT = 0x182


@dataclass
class SideStats:
    name: str
    can_id: int
    last_mono: float | None = None
    last_wall: float | None = None
    count: int = 0
    gap_count: int = 0
    max_gap_s: float = 0.0
    in_gap: bool = False


@dataclass
class Monitor:
    gap_s: float
    sides: dict[int, SideStats] = field(default_factory=dict)
    log_path: Path | None = None
    _log_fp: object | None = None
    _log_writer: csv.writer | None = None

    def __post_init__(self) -> None:
        self.sides = {
            ID_LEFT: SideStats("left", ID_LEFT),
            ID_RIGHT: SideStats("right", ID_RIGHT),
        }
        if self.log_path is not None:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            new_file = not self.log_path.exists()
            self._log_fp = self.log_path.open("a", newline="", encoding="utf-8")
            self._log_writer = csv.writer(self._log_fp)
            if new_file:
                self._log_writer.writerow(
                    ["wall_iso", "event", "side", "can_id", "gap_s", "count"]
                )

    def close(self) -> None:
        if self._log_fp is not None:
            self._log_fp.close()
            self._log_fp = None
            self._log_writer = None

    def _write_log(
        self, event: str, side: SideStats, gap_s: float | None = None
    ) -> None:
        if self._log_writer is None:
            return
        wall = datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")
        self._log_writer.writerow(
            [
                wall,
                event,
                side.name,
                f"0x{side.can_id:03X}",
                f"{gap_s:.3f}" if gap_s is not None else "",
                side.count,
            ]
        )
        self._log_fp.flush()  # type: ignore[union-attr]

    def on_frame(self, can_id: int, wall: float, mono: float) -> None:
        side = self.sides.get(can_id)
        if side is None:
            return

        if side.last_mono is not None:
            interval = mono - side.last_mono
            if interval > side.max_gap_s:
                side.max_gap_s = interval
            if side.in_gap:
                # 直前フレームからの沈黙時間（復帰時点）
                gap_len = interval
                side.in_gap = False
                msg = (
                    f"[RECOVER] {side.name} 0x{side.can_id:03X} "
                    f"途絶 {gap_len:.3f}s のあと復帰 (frames={side.count + 1})"
                )
                print(msg, flush=True)
                self._write_log("recover", side, gap_len)

        side.last_mono = mono
        side.last_wall = wall
        side.count += 1
        if side.count == 1:
            print(
                f"[START]  {side.name} 0x{side.can_id:03X} 受信開始",
                flush=True,
            )
            self._write_log("start", side)

    def poll_gaps(self, mono: float) -> None:
        for side in self.sides.values():
            if side.last_mono is None:
                continue
            silent = mono - side.last_mono
            if silent >= self.gap_s and not side.in_gap:
                side.in_gap = True
                side.gap_count += 1
                if silent > side.max_gap_s:
                    side.max_gap_s = silent
                other = self.sides[ID_RIGHT if side.can_id == ID_LEFT else ID_LEFT]
                one_sided = (
                    other.last_mono is not None
                    and (mono - other.last_mono) < self.gap_s
                )
                kind = "片側途絶" if one_sided else "途絶"
                msg = (
                    f"[GAP]    {side.name} 0x{side.can_id:03X} {kind} "
                    f"{silent:.3f}s (threshold={self.gap_s:.3f}s, "
                    f"frames={side.count})"
                )
                print(msg, flush=True)
                self._write_log("gap", side, silent)
            elif side.in_gap and silent > side.max_gap_s:
                side.max_gap_s = silent

    def summary(self) -> None:
        print("--- summary ---", flush=True)
        for side in self.sides.values():
            last = (
                datetime.fromtimestamp(side.last_wall).isoformat(timespec="milliseconds")
                if side.last_wall is not None
                else "-"
            )
            print(
                f"  {side.name} 0x{side.can_id:03X}: frames={side.count} "
                f"gaps={side.gap_count} max_gap={side.max_gap_s:.3f}s "
                f"last={last} in_gap={side.in_gap}",
                flush=True,
            )


def open_can(iface: str) -> socket.socket:
    sock = socket.socket(AF_CAN, socket.SOCK_RAW, CAN_RAW)
    # (can_id, can_mask) を左右分
    filt = struct.pack("=II", ID_LEFT, CAN_SFF_MASK) + struct.pack(
        "=II", ID_RIGHT, CAN_SFF_MASK
    )
    sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER, filt)
    sock.bind((iface,))
    sock.setblocking(False)
    return sock


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Monitor 0x181/0x182 torque TPDO gaps")
    p.add_argument("--iface", default="can0", help="CAN interface (default: can0)")
    p.add_argument(
        "--gap",
        type=float,
        default=1.0,
        help="途絶とみなす秒数 (default: 1.0)。モーター再起動閾値は 3s",
    )
    p.add_argument(
        "--poll",
        type=float,
        default=0.05,
        help="途絶判定の周期秒 (default: 0.05)",
    )
    p.add_argument(
        "--status",
        type=float,
        default=10.0,
        help="生存表示の間隔秒。0 で無効 (default: 10)",
    )
    p.add_argument(
        "--log",
        type=Path,
        default=None,
        help="検知イベントを追記する CSV パス",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    if args.gap <= 0:
        print("--gap は正の数にしてください", file=sys.stderr)
        return 2

    stop = False

    def _on_signal(signum: int, _frame: object) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        sock = open_can(args.iface)
    except OSError as exc:
        print(f"CAN open failed ({args.iface}): {exc}", file=sys.stderr)
        return 1

    mon = Monitor(gap_s=args.gap, log_path=args.log)
    print(
        f"watching {args.iface} 0x181/0x182 gap>={args.gap:.3f}s "
        f"(Ctrl+C to stop)",
        flush=True,
    )

    last_status = time.monotonic()
    try:
        while not stop:
            ready, _, _ = select.select([sock], [], [], args.poll)
            if ready:
                try:
                    data = sock.recv(CAN_FRAME_SIZE)
                except BlockingIOError:
                    data = b""
                if len(data) >= CAN_FRAME_SIZE:
                    can_id, dlc, _payload = struct.unpack(
                        CAN_FRAME_FMT, data[:CAN_FRAME_SIZE]
                    )
                    can_id &= CAN_SFF_MASK
                    if dlc > 0 and can_id in mon.sides:
                        mon.on_frame(can_id, time.time(), time.monotonic())

            mon.poll_gaps(time.monotonic())

            if args.status > 0 and (time.monotonic() - last_status) >= args.status:
                last_status = time.monotonic()
                l = mon.sides[ID_LEFT]
                r = mon.sides[ID_RIGHT]
                print(
                    f"[STATUS] left={l.count} right={r.count} "
                    f"gaps L={l.gap_count}/R={r.gap_count} "
                    f"max L={l.max_gap_s:.3f}s R={r.max_gap_s:.3f}s",
                    flush=True,
                )
    finally:
        mon.summary()
        mon.close()
        sock.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
