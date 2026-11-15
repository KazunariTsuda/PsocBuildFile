#!/usr/bin/env python3
"""ラズパイ自身の温度、CPU、CAN の状態を CSV に残す。

起動のたびに新しいファイルを作る。1行は --interval 秒ごと（既定 10 秒）。
温度、5V 入力電圧、CPU 使用率、CAN の状態を残す。
  log/pi_status_YYYYMMDD_HHMMSS.csv

CAN インターフェースは上げたり下げたりしない。PSoC が出す 0x092 が
届いていれば、その TEC/REC も同じ行に載せる。

  python3 pi_status_log.py
  python3 pi_status_log.py --interval 10 --dir log

起動時からの自動実行は pi_status_log.sh を使う。
"""

from __future__ import annotations

import argparse
import csv
import os
import re
import select
import signal
import socket
import struct
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

AF_CAN = getattr(socket, "AF_CAN", 29)
CAN_RAW = getattr(socket, "CAN_RAW", 1)
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
SOL_CAN_RAW = 101
CAN_RAW_FILTER = 1
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)
ID_HEALTH = 0x092

RE_STATE = re.compile(r"can state (\S+)")
RE_BERR = re.compile(r"berr-counter tx (\d+) rx (\d+)")
RE_VOLT = re.compile(r"=\s*([0-9]+(?:\.[0-9]+)?)V")
RE_STATS = re.compile(
    r"re-started\s+bus-errors\s+arbit-lost\s+error-warn\s+error-pass\s+bus-off\s+"
    r"(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)",
    re.S,
)

THROTTLE_BITS = (
    (1 << 0, "電圧低下"),
    (1 << 1, "周波数制限"),
    (1 << 2, "温度制限"),
    (1 << 3, "ソフト温度"),
    (1 << 16, "電圧低下の履歴"),
    (1 << 17, "周波数制限の履歴"),
    (1 << 18, "温度制限の履歴"),
    (1 << 19, "ソフト温度の履歴"),
)

COLUMNS = [
    "pi_time",
    "uptime_s",
    "cpu_temp_c",
    "input_v",
    "cpu_pct",
    "load1",
    "mem_used_pct",
    "throttled",
    "can_link",
    "can_state",
    "can_tec",
    "can_rec",
    "can_restarts",
    "can_bus_off",
    "psoc_tec",
    "psoc_rec",
    "psoc_busoff",
    "psoc_flags",
    "psoc_age_s",
]


def throttle_text(value: int) -> str:
    names = [label for bit, label in THROTTLE_BITS if value & bit]
    return " ".join(names) if names else "なし"


def read_uptime() -> str:
    try:
        return Path("/proc/uptime").read_text().split()[0]
    except (OSError, IndexError):
        return ""


def read_cpu_temp() -> str:
    try:
        milli = int(Path("/sys/class/thermal/thermal_zone0/temp").read_text().strip())
    except (OSError, ValueError):
        return ""
    return f"{milli / 1000.0:.1f}"


def parse_volts(text: str) -> str:
    matched = RE_VOLT.search(text)
    if matched is None:
        return ""
    return f"{float(matched.group(1)):.3f}"


def read_input_voltage() -> str:
    """5V 入力。Pi 4/5 の PMIC が返す EXT5V_V。"""
    try:
        proc = subprocess.run(
            ["vcgencmd", "pmic_read_adc", "EXT5V_V"],
            capture_output=True,
            text=True,
            timeout=0.4,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return ""
    if proc.returncode != 0:
        return ""
    return parse_volts(proc.stdout)


def read_load1() -> str:
    try:
        return Path("/proc/loadavg").read_text().split()[0]
    except (OSError, IndexError):
        return ""


def read_mem_used_pct() -> str:
    info: dict[str, int] = {}
    try:
        lines = Path("/proc/meminfo").read_text().splitlines()
    except OSError:
        return ""
    for line in lines:
        key, _, rest = line.partition(":")
        parts = rest.split()
        if parts:
            try:
                info[key] = int(parts[0])
            except ValueError:
                continue
    total = info.get("MemTotal", 0)
    avail = info.get("MemAvailable", 0)
    if total <= 0:
        return ""
    return f"{(total - avail) * 100.0 / total:.1f}"


def read_cpu_jiffies() -> tuple[int, int] | None:
    try:
        parts = Path("/proc/stat").read_text().splitlines()[0].split()[1:]
        nums = [int(item) for item in parts]
    except (OSError, ValueError, IndexError):
        return None
    idle = nums[3] + (nums[4] if len(nums) > 4 else 0)
    return sum(nums), idle


def cpu_percent(previous: tuple[int, int] | None, current: tuple[int, int] | None) -> str:
    if previous is None or current is None:
        return ""
    total = current[0] - previous[0]
    idle = current[1] - previous[1]
    if total <= 0:
        return ""
    return f"{(total - idle) * 100.0 / total:.1f}"


def read_throttled() -> tuple[str, str]:
    try:
        proc = subprocess.run(
            ["vcgencmd", "get_throttled"],
            capture_output=True,
            text=True,
            timeout=0.4,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return "", ""
    if proc.returncode != 0:
        return "", ""
    text = proc.stdout.strip().split("=")[-1].strip()
    try:
        value = int(text, 0)
    except ValueError:
        return "", ""
    return f"0x{value:x}", throttle_text(value)


def link_text(iface: str) -> str:
    try:
        proc = subprocess.run(
            ["ip", "-details", "-statistics", "link", "show", iface],
            capture_output=True,
            text=True,
            timeout=0.8,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return ""
    return (proc.stdout or "") + (proc.stderr or "")


def parse_can_link(text: str) -> dict[str, str]:
    row = {
        "can_link": "",
        "can_state": "",
        "can_tec": "",
        "can_rec": "",
        "can_restarts": "",
        "can_bus_off": "",
    }
    if not text:
        return row
    if "does not exist" in text or "Cannot find device" in text:
        row["can_link"] = "なし"
        return row
    first = text.split("\n", 1)[0]
    if " state UP " in first:
        row["can_link"] = "UP"
    elif " state DOWN " in first:
        row["can_link"] = "DOWN"
    state = RE_STATE.search(text)
    if state:
        row["can_state"] = state.group(1)
    berr = RE_BERR.search(text)
    if berr:
        row["can_tec"] = berr.group(1)
        row["can_rec"] = berr.group(2)
    stats = RE_STATS.search(text)
    if stats:
        row["can_restarts"] = stats.group(1)
        row["can_bus_off"] = stats.group(6)
    return row


def open_health_socket(iface: str) -> socket.socket | None:
    try:
        sock = socket.socket(AF_CAN, socket.SOCK_RAW, CAN_RAW)
        sock.setblocking(False)
        filt = struct.pack("=II", ID_HEALTH, CAN_SFF_MASK)
        sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER, filt)
        sock.bind((iface,))
    except OSError:
        return None
    return sock


def take_health(sock: socket.socket | None) -> bytes | None:
    if sock is None:
        return None
    latest: bytes | None = None
    while True:
        try:
            frame = sock.recv(CAN_FRAME_SIZE)
        except (BlockingIOError, InterruptedError):
            break
        except OSError:
            break
        can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, frame[:CAN_FRAME_SIZE])
        if can_id & CAN_ERR_FLAG:
            continue
        if (can_id & CAN_SFF_MASK) == ID_HEALTH and dlc >= 6:
            latest = data[:dlc]
    return latest


def health_fields(data: bytes | None, seen_at: float | None, now: float) -> dict[str, str]:
    row = {"psoc_tec": "", "psoc_rec": "", "psoc_busoff": "", "psoc_flags": "", "psoc_age_s": ""}
    if data is None or seen_at is None:
        return row
    row["psoc_tec"] = str(data[0])
    row["psoc_rec"] = str(data[1])
    row["psoc_busoff"] = str((data[2] << 8) | data[3])
    row["psoc_flags"] = f"0x{data[5]:02x}"
    row["psoc_age_s"] = f"{now - seen_at:.1f}"
    return row


def open_csv(directory: Path) -> tuple[object, csv.DictWriter, Path]:
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    path = directory / f"pi_status_{stamp}.csv"
    handle = path.open("w", encoding="utf-8", newline="")
    writer = csv.DictWriter(handle, fieldnames=COLUMNS)
    writer.writeheader()
    handle.flush()
    os.fsync(handle.fileno())
    return handle, writer, path


def write_row(handle, writer: csv.DictWriter, row: dict[str, str]) -> None:
    writer.writerow(row)
    handle.flush()
    os.fsync(handle.fileno())


def sample(
    iface: str,
    previous_cpu: tuple[int, int] | None,
    health: bytes | None,
    health_at: float | None,
) -> tuple[dict[str, str], tuple[int, int] | None]:
    now = time.monotonic()
    cpu = read_cpu_jiffies()
    throttled, throttled_label = read_throttled()
    row = {
        "pi_time": datetime.now().astimezone().isoformat(timespec="seconds"),
        "uptime_s": read_uptime(),
        "cpu_temp_c": read_cpu_temp(),
        "input_v": read_input_voltage(),
        "cpu_pct": cpu_percent(previous_cpu, cpu),
        "load1": read_load1(),
        "mem_used_pct": read_mem_used_pct(),
        "throttled": throttled_label if throttled else "",
    }
    if throttled:
        row["throttled"] = f"{throttled} {throttled_label}"
    row.update(parse_can_link(link_text(iface)))
    row.update(health_fields(health, health_at, now))
    return row, cpu


def main() -> int:
    parser = argparse.ArgumentParser(description="ラズパイの温度、CPU、CAN を CSV に記録する")
    parser.add_argument("--dir", default="log", help="CSV を置くディレクトリ。無ければ作る")
    parser.add_argument("--interval", type=float, default=10.0, help="記録間隔（秒）")
    parser.add_argument("--iface", default="can0")
    args = parser.parse_args()
    if args.interval < 1.0:
        parser.error("--interval は 1 以上")

    handle, writer, path = open_csv(Path(args.dir))
    print(f"記録先: {path}", flush=True)
    print(f"間隔: {args.interval:.0f} 秒  CAN: {args.iface}", flush=True)
    print("止めるときは Ctrl+C", flush=True)

    health_sock = open_health_socket(args.iface)
    health: bytes | None = None
    health_at: float | None = None
    cpu = read_cpu_jiffies()
    time.sleep(0.2)

    def request_stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    try:
        while True:
            if health_sock is not None:
                ready, _, _ = select.select([health_sock], [], [], 0.0)
                if ready:
                    latest = take_health(health_sock)
                    if latest is not None:
                        health = latest
                        health_at = time.monotonic()
            row, cpu = sample(args.iface, cpu, health, health_at)
            write_row(handle, writer, row)
            print(
                f"{row['pi_time']}  temp={row['cpu_temp_c']}  input={row['input_v']}V  cpu={row['cpu_pct']}%  "
                f"can={row['can_link']} {row['can_state']} tec={row['can_tec']} rec={row['can_rec']}",
                flush=True,
            )
            deadline = time.monotonic() + args.interval
            while True:
                left = deadline - time.monotonic()
                if left <= 0:
                    break
                if health_sock is None:
                    time.sleep(left)
                    break
                ready, _, _ = select.select([health_sock], [], [], min(left, 1.0))
                if ready:
                    latest = take_health(health_sock)
                    if latest is not None:
                        health = latest
                        health_at = time.monotonic()
    except KeyboardInterrupt:
        return 0
    finally:
        handle.flush()
        os.fsync(handle.fileno())
        handle.close()
        if health_sock is not None:
            health_sock.close()


if __name__ == "__main__":
    sys.exit(main())
