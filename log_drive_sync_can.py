#!/usr/bin/env python3
"""駆動コマンド(0x070)と SYNC(0x080) を candump -ta 相当で CSV に残す。

  candump -ta can0,070:7FF,080:7FF
  と同じ ID を受け、絶対時刻付きで書き出す。走行アプリとは別プロセスで動かす。

  python3 log_drive_sync_can.py
  python3 log_drive_sync_can.py --iface can0 --out drive_sync.csv
  python3 log_drive_sync_can.py --echo
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
from datetime import datetime, timezone
from pathlib import Path

AF_CAN = getattr(socket, "AF_CAN", 29)
CAN_RAW = getattr(socket, "CAN_RAW", 1)
SOL_CAN_RAW = 101
CAN_RAW_FILTER = 1
CAN_SFF_MASK = 0x000007FF
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

ID_DRIVE = 0x070
ID_SYNC = 0x080


def kind_of(can_id: int, dlc: int) -> str:
    if can_id == ID_DRIVE:
        return "drive"
    if can_id == ID_SYNC and dlc == 0:
        return "sync"
    if can_id == ID_SYNC:
        return "sync_nonempty"
    return "other"


def open_can(iface: str) -> socket.socket:
    sock = socket.socket(AF_CAN, socket.SOCK_RAW, CAN_RAW)
    filt = struct.pack("=II", ID_DRIVE, CAN_SFF_MASK) + struct.pack(
        "=II", ID_SYNC, CAN_SFF_MASK
    )
    sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER, filt)
    sock.bind((iface,))
    sock.setblocking(False)
    return sock


def default_out_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(f"drive_sync_{stamp}.csv")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Log can0 0x070/0x080 to CSV (candump -ta equivalent)"
    )
    p.add_argument("--iface", default="can0", help="CAN interface (default: can0)")
    p.add_argument(
        "--out",
        type=Path,
        default=None,
        help="出力 CSV。省略時は drive_sync_YYYYMMDD_HHMMSS.csv",
    )
    p.add_argument(
        "--echo",
        action="store_true",
        help="candump 風に標準出力にも出す",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    out_path = args.out if args.out is not None else default_out_path()

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

    out_path.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    print(
        f"logging {args.iface} 0x070/0x080 -> {out_path} (Ctrl+C to stop)",
        flush=True,
    )

    try:
        with out_path.open("w", newline="", encoding="utf-8") as fp:
            writer = csv.writer(fp)
            writer.writerow(
                [
                    "wall_epoch",
                    "wall_iso",
                    "iface",
                    "can_id",
                    "dlc",
                    "data_hex",
                    "kind",
                ]
            )
            fp.flush()

            while not stop:
                ready, _, _ = select.select([sock], [], [], 0.2)
                if not ready:
                    continue
                try:
                    raw = sock.recv(CAN_FRAME_SIZE)
                except BlockingIOError:
                    continue
                if len(raw) < CAN_FRAME_SIZE:
                    continue

                can_id, dlc, payload = struct.unpack(
                    CAN_FRAME_FMT, raw[:CAN_FRAME_SIZE]
                )
                can_id &= CAN_SFF_MASK
                if can_id not in (ID_DRIVE, ID_SYNC):
                    continue
                if dlc > 8:
                    dlc = 8

                wall = time.time()
                wall_iso = (
                    datetime.fromtimestamp(wall, tz=timezone.utc)
                    .astimezone()
                    .isoformat(timespec="microseconds")
                )
                data = payload[:dlc]
                data_hex = " ".join(f"{b:02X}" for b in data)
                kind = kind_of(can_id, dlc)

                writer.writerow(
                    [
                        f"{wall:.6f}",
                        wall_iso,
                        args.iface,
                        f"{can_id:03X}",
                        dlc,
                        data_hex,
                        kind,
                    ]
                )
                fp.flush()
                count += 1

                if args.echo:
                    # candump -ta に近い表示
                    print(
                        f" ({wall:.6f})  {args.iface}  {can_id:03X}   [{dlc}]"
                        + (f"  {data_hex}" if dlc else ""),
                        flush=True,
                    )
    finally:
        sock.close()
        print(f"stopped. frames={count} file={out_path}", flush=True)

    return 0


if __name__ == "__main__":
    sys.exit(main())
