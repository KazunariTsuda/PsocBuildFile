#!/usr/bin/env python3
"""PSoC の UART と、ラズパイ自身の状態を、別フォルダの CSV に残す。

起動のたびにファイルを 1 つずつ作る。
  log/uart/psoc_uart_YYYYMMDD_HHMMSS.csv
  log/status/pi_status_YYYYMMDD_HHMMSS.csv

UART は、先に PSoC の EEPROM を読み、そのあと届いた行を足す。
状態は --interval 秒ごと（既定 10 秒）。温度、5V 電圧、CPU 使用率、CAN。
各フォルダは新しい 100 件だけ残し、それより古いファイルは消す。

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
import socket
import struct
import subprocess
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


KEEP_FILES = 100

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
STATUS_COLUMNS = [
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


def prune_old(directory: Path, prefix: str, current: Path) -> None:
    """同じ種類の CSV を、いまのファイルを含めて 100 件残す。"""
    current_resolved = current.resolve()
    others = []
    for path in directory.glob(f"{prefix}_*.csv"):
        if path.is_file() and path.resolve() != current_resolved:
            others.append(path)
    others.sort(key=lambda path: path.stat().st_mtime, reverse=True)
    for path in others[KEEP_FILES - 1:]:
        try:
            path.unlink()
        except OSError:
            pass


def open_uart_csv(directory: Path) -> tuple[object, csv.writer, Path]:
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    path = directory / f"psoc_uart_{stamp}.csv"
    handle = path.open("w", encoding="utf-8", newline="")
    writer = csv.writer(handle)
    writer.writerow(["pi_time", "psoc_time", "message"])
    handle.flush()
    os.fsync(handle.fileno())
    prune_old(directory, "psoc_uart", path)
    return handle, writer, path


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


def open_status_csv(directory: Path) -> tuple[object, csv.DictWriter, Path]:
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    path = directory / f"pi_status_{stamp}.csv"
    handle = path.open("w", encoding="utf-8", newline="")
    writer = csv.DictWriter(handle, fieldnames=STATUS_COLUMNS)
    writer.writeheader()
    handle.flush()
    os.fsync(handle.fileno())
    prune_old(directory, "pi_status", path)
    return handle, writer, path


def write_status(handle, writer: csv.DictWriter, row: dict[str, str]) -> None:
    writer.writerow(row)
    handle.flush()
    os.fsync(handle.fileno())


def sample_status(
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
        "throttled": f"{throttled} {throttled_label}" if throttled else "",
    }
    row.update(parse_can_link(link_text(iface)))
    row.update(health_fields(health, health_at, now))
    return row, cpu


def record_uart(handle, writer, text: str) -> None:
    if text in ("LOGBEGIN", "LOGEND") or DUMP_LINE.match(text):
        return
    psoc_time, message = split_line(text)
    pi_time = datetime.now().astimezone().isoformat(timespec="milliseconds")
    write_row(handle, writer, pi_time, psoc_time, message)
    print(text, flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="PSoC UART とラズパイの状態を CSV に記録する")
    parser.add_argument("--port", help="例: /dev/ttyUSB0。省略時は CP210x を探す")
    parser.add_argument("--dir", default="log", help="この下に uart と status フォルダを作る")
    parser.add_argument("--interval", type=float, default=10.0, help="状態の記録間隔（秒）")
    parser.add_argument("--iface", default="can0")
    args = parser.parse_args()
    if args.interval < 1.0:
        parser.error("--interval は 1 以上")

    root = Path(args.dir)
    port = args.port or find_port()
    uart_handle, uart_writer, uart_path = open_uart_csv(root / "uart")
    status_handle, status_writer, status_path = open_status_csv(root / "status")
    print(f"UART: {uart_path}", flush=True)
    print(f"状態: {status_path}", flush=True)
    print(f"ポート: {port}  115200 8N1  状態は {args.interval:.0f} 秒ごと", flush=True)

    fd = open_serial(port)
    health_sock = open_health_socket(args.iface)
    health: bytes | None = None
    health_at: float | None = None
    cpu = read_cpu_jiffies()
    pending, early = pull_stored(fd, uart_handle, uart_writer)
    print("このあとの行を記録します。止めるときは Ctrl+C", flush=True)
    for pi_time, psoc_time, message in early:
        write_row(uart_handle, uart_writer, pi_time, psoc_time, message)
    next_status = time.monotonic() + 0.2
    watch = [fd] if health_sock is None else [fd, health_sock]

    def request_stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    try:
        while True:
            now = time.monotonic()
            if health_sock is not None:
                latest = take_health(health_sock)
                if latest is not None:
                    health = latest
                    health_at = now
            if now >= next_status:
                row, cpu = sample_status(args.iface, cpu, health, health_at)
                write_status(status_handle, status_writer, row)
                print(
                    f"{row['pi_time']}  temp={row['cpu_temp_c']}  input={row['input_v']}V  "
                    f"cpu={row['cpu_pct']}%  can={row['can_link']} {row['can_state']} "
                    f"tec={row['can_tec']} rec={row['can_rec']}",
                    flush=True,
                )
                next_status = time.monotonic() + args.interval
            timeout = max(0.0, next_status - time.monotonic())
            ready, _, _ = select.select(watch, [], [], timeout)
            if fd not in ready:
                continue
            chunk = os.read(fd, 256)
            if not chunk:
                continue
            pending.extend(chunk)
            for text in take_lines(pending):
                record_uart(uart_handle, uart_writer, text)
    except KeyboardInterrupt:
        return 0
    finally:
        for handle in (uart_handle, status_handle):
            handle.flush()
            os.fsync(handle.fileno())
            handle.close()
        os.close(fd)
        if health_sock is not None:
            health_sock.close()


if __name__ == "__main__":
    sys.exit(main())
