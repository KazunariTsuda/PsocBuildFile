#!/usr/bin/env python3
"""PSoC の USB-UART（CP210x、115200 8N1）を CSV に残す。

起動のたびに新しいファイルを作る。
先に PSoC の EEPROM に残っている動作ログを読み出し、そのあと届いた行を足す。
CSV は log フォルダに置く。フォルダが無ければ作る。
  log/psoc_uart_YYYYMMDD_HHMMSS.csv

列は次の3つ。
  pi_time    行を受け取った瞬間のラズパイ時刻（タイムゾーン付き）
  psoc_time  保存ログは日付付きの JST、未同期は +ミリ秒、古い記録は raw:。
             起動後の行は HH/MM/SS または未同期の --/--/--
  message    時刻を除いた残りのログ

1行書くたびに flush と fsync する。ラズパイが落ちても、書き終えた行は残る。

使い方:
  python3 uart_csv_log.py
  python3 uart_csv_log.py --port /dev/ttyUSB0 --dir log

起動時の自動実行と、PSoC 更新前の停止は psoc_uart_log.sh を使う。
"""

from __future__ import annotations

import argparse
import csv
import os
import re
import select
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

PSOC_TIME = re.compile(r"^(?P<psoc>(?:\d{2}/\d{2}/\d{2})|(?:--/--/--))\s*(?P<rest>.*)$")
DUMP_LINE = re.compile(r"^LOGDUMP\s+(?P<psoc>\S+)\s*(?P<rest>.*)$")


def split_line(text: str) -> tuple[str, str]:
    """行頭が PSoC 時刻なら分離する。無ければ時刻は空で、本文は行全体。"""
    matched = PSOC_TIME.match(text.strip())
    if matched is None:
        return "", text.strip()
    return matched.group("psoc"), matched.group("rest")


def find_port() -> str:
    by_id = Path("/dev/serial/by-id")
    hits: list[Path] = []
    if by_id.is_dir():
        for path in sorted(by_id.iterdir()):
            name = path.name.lower()
            if "cp210" in name or "silicon_labs" in name or "siliconlabs" in name:
                hits.append(path)
    if len(hits) == 1:
        return str(hits[0])
    if len(hits) > 1:
        names = "\n".join(str(path) for path in hits)
        raise SystemExit("CP210x が複数あります。--port で指定してください。\n" + names)
    usbs = sorted(Path("/dev").glob("ttyUSB*"))
    if len(usbs) == 1:
        return str(usbs[0])
    raise SystemExit("CP210x が見つかりません。--port で /dev/ttyUSB* を指定してください。")


def open_serial(path: str) -> int:
    import termios

    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    # tcgetattr の 4 が入力速度、5 が出力速度。cfsetispeed が無い Python でも同じ設定になる
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def write_row(handle, writer, pi_time: str, psoc_time: str, message: str) -> None:
    writer.writerow([pi_time, psoc_time, message])
    handle.flush()
    os.fsync(handle.fileno())


def send_log_request(fd: int) -> None:
    """受信側は 4 バイトしか溜められないので、LOG を 1 バイトずつ送る。"""
    for byte in b"LOG\n":
        os.write(fd, bytes([byte]))
        time.sleep(0.008)


def take_lines(pending: bytearray) -> list[str]:
    lines: list[str] = []
    while True:
        cut = pending.find(b"\n")
        if cut < 0:
            break
        raw = pending[:cut]
        del pending[: cut + 1]
        text = raw.decode("utf-8", errors="replace").rstrip("\r").strip()
        if text != "" and text != "LOG":
            lines.append(text)
    return lines


def pull_stored(fd: int, handle, writer) -> tuple[bytearray, list[tuple[str, str, str]]]:
    """EEPROM のログを先に CSV へ書く。届いていた通常行はあとで足す。"""
    pending = bytearray()
    early: list[tuple[str, str, str]] = []
    begun = False
    stored = 0
    begin_deadline = time.monotonic() + 15.0
    next_send = 0.0
    end_deadline = None
    print("保存ログを読み出します", flush=True)

    while True:
        now = time.monotonic()
        if not begun and now >= next_send:
            send_log_request(fd)
            next_send = now + 5.0
        if not begun and now >= begin_deadline:
            print("保存ログは取れませんでした。このあとの行だけ記録します。", flush=True)
            break
        if begun and end_deadline is not None and now >= end_deadline:
            print("保存ログの終わりが来ませんでした。取れた分だけ残します。", flush=True)
            break
        ready, _, _ = select.select([fd], [], [], 0.2)
        if not ready:
            continue
        chunk = os.read(fd, 256)
        if not chunk:
            continue
        pending.extend(chunk)
        for text in take_lines(pending):
            if text == "LOGBEGIN":
                begun = True
                end_deadline = time.monotonic() + 10.0
                continue
            if text == "LOGEND":
                print(f"保存ログ: {stored} 件", flush=True)
                return pending, early
            matched = DUMP_LINE.match(text)
            if begun and matched is not None:
                pi_time = datetime.now().astimezone().isoformat(timespec="milliseconds")
                write_row(handle, writer, pi_time, matched.group("psoc"), matched.group("rest"))
                stored += 1
                end_deadline = time.monotonic() + 10.0
                continue
            psoc_time, message = split_line(text)
            pi_time = datetime.now().astimezone().isoformat(timespec="milliseconds")
            early.append((pi_time, psoc_time, message))
    return pending, early


def open_csv(directory: Path) -> tuple[object, csv.writer, Path]:
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    path = directory / f"psoc_uart_{stamp}.csv"
    handle = path.open("w", encoding="utf-8", newline="")
    writer = csv.writer(handle)
    writer.writerow(["pi_time", "psoc_time", "message"])
    handle.flush()
    os.fsync(handle.fileno())
    return handle, writer, path


def main() -> int:
    parser = argparse.ArgumentParser(description="PSoC UART を CSV に記録する")
    parser.add_argument("--port", help="例: /dev/ttyUSB0。省略時は CP210x を探す")
    parser.add_argument("--dir", default="log", help="CSV を置くディレクトリ。無ければ作る")
    args = parser.parse_args()

    port = args.port or find_port()
    handle, writer, path = open_csv(Path(args.dir))
    print(f"記録先: {path}", flush=True)
    print(f"ポート: {port}  115200 8N1", flush=True)

    fd = open_serial(port)
    pending, early = pull_stored(fd, handle, writer)
    for pi_time, psoc_time, message in early:
        write_row(handle, writer, pi_time, psoc_time, message)

    def request_stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    try:
        while True:
            chunk = os.read(fd, 256)
            if not chunk:
                continue
            pending.extend(chunk)
            for text in take_lines(pending):
                if text in ("LOGBEGIN", "LOGEND") or DUMP_LINE.match(text):
                    continue
                psoc_time, message = split_line(text)
                pi_time = datetime.now().astimezone().isoformat(timespec="milliseconds")
                write_row(handle, writer, pi_time, psoc_time, message)
    except KeyboardInterrupt:
        return 0
    finally:
        handle.flush()
        os.fsync(handle.fileno())
        handle.close()
        os.close(fd)


if __name__ == "__main__":
    sys.exit(main())
