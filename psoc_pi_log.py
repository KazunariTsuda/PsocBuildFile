#!/usr/bin/env python3
"""PSoC の UART と、ラズパイの CAN・電源・カーネルを CSV に残す。

走行アプリとは別プロセスで動かす。アプリが bus-off で止まっても記録は続ける。
起動のたびに、開始時刻の付いたファイルを新しく作る。

  log/uart/psoc_uart_YYYYMMDD_HHMMSS.csv      受信した行
  log/status/pi_status_YYYYMMDD_HHMMSS.csv    10 秒ごとの状態
  log/can/can_event_YYYYMMDD_HHMMSS.csv       ビットタイミング、CAN の変化、エラーフレーム、アプリの起動と終了
  log/kernel/kern_YYYYMMDD_HHMMSS.csv         カーネルの落ちた理由と CAN。起動時に前回分もコピー

各フォルダは 100 ファイルまで。超えたら名前の古いものから消す。

pi_time は現地時刻にオフセットを付けた ISO 時刻。ミリ秒まで。
UART は届いた順に書く。並べるときは pi_time を使う。

  python3 psoc_pi_log.py
  python3 psoc_pi_log.py --port /dev/serial/by-id/usb-Silicon_Labs_... --dir log
  python3 psoc_pi_log.py --self-test
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
import tempfile
import time
from datetime import datetime
from pathlib import Path

PSOC_TIME = re.compile(
    r"^(?P<psoc>(?:\d{2}[/:]\d{2}[/:]\d{2})|(?:--[/:]--[/:]--))\s*(?P<rest>.*)$"
)
CLOCK_FACE = re.compile(r"^(?P<h>\d{2})[/:](?P<m>\d{2})[/:](?P<s>\d{2})$")
DUMP_LINE = re.compile(r"^LOGDUMP\s+(?P<psoc>\S+)\s*(?P<rest>.*)$")
DUMP_STS = re.compile(
    r"^(?P<s1>[0-9A-Fa-f]{2}),(?P<s2>[0-9A-Fa-f]{2}),(?P<s3>[0-9A-Fa-f]{2}),(?P<s4>[0-9A-Fa-f]{2})(?: (?P<text>.*))?$"
)

AF_CAN = getattr(socket, "AF_CAN", 29)
CAN_RAW = getattr(socket, "CAN_RAW", 1)
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
SOL_CAN_RAW = 101
CAN_RAW_FILTER = 1
CAN_RAW_ERR_FILTER = 2
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

CAN_ERR_TX_TIMEOUT = 0x00000001
CAN_ERR_LOSTARB = 0x00000002
CAN_ERR_CRTL = 0x00000004
CAN_ERR_PROT = 0x00000008
CAN_ERR_TRX = 0x00000010
CAN_ERR_ACK = 0x00000020
CAN_ERR_BUSOFF = 0x00000040
CAN_ERR_BUSERROR = 0x00000080
CAN_ERR_RESTARTED = 0x00000100
CAN_ERR_MASK = 0x1FFFFFFF
CAN_ERR_CRTL_RX_OVERFLOW = 0x01
CAN_ERR_CRTL_TX_OVERFLOW = 0x02
CAN_ERR_CRTL_RX_WARNING = 0x04
CAN_ERR_CRTL_TX_WARNING = 0x08
CAN_ERR_CRTL_RX_PASSIVE = 0x10
CAN_ERR_CRTL_TX_PASSIVE = 0x20
CAN_ERR_CRTL_ACTIVE = 0x40

CLASS_BITS = (
    (CAN_ERR_TX_TIMEOUT, "TX_TIMEOUT"),
    (CAN_ERR_LOSTARB, "LOSTARB"),
    (CAN_ERR_CRTL, "CRTL"),
    (CAN_ERR_PROT, "PROT"),
    (CAN_ERR_TRX, "TRX"),
    (CAN_ERR_ACK, "ACK"),
    (CAN_ERR_BUSOFF, "BUSOFF"),
    (CAN_ERR_BUSERROR, "BUSERR"),
    (CAN_ERR_RESTARTED, "RESTART"),
)
PROT_BITS = (
    (0x01, "bit"),
    (0x02, "form"),
    (0x04, "stuff"),
    (0x08, "bit0"),
    (0x10, "bit1"),
    (0x20, "overload"),
    (0x40, "active"),
    (0x80, "tx"),
)
PROT_LOC = {
    0x00: "UNSPEC",
    0x02: "ID28_21",
    0x03: "SOF",
    0x04: "SRTR",
    0x05: "IDE",
    0x06: "ID20_18",
    0x07: "ID17_13",
    0x08: "CRC_SEQ",
    0x09: "RES0",
    0x0A: "DATA",
    0x0B: "DLC",
    0x0C: "RTR",
    0x0D: "RES1",
    0x0E: "ID04_00",
    0x0F: "ID12_05",
    0x12: "INTERM",
    0x18: "CRC_DEL",
    0x19: "ACK",
    0x1A: "EOF",
    0x1B: "ACK_DEL",
}
CTRL_BITS = (
    (CAN_ERR_CRTL_TX_PASSIVE, "TX_PASSIVE"),
    (CAN_ERR_CRTL_RX_PASSIVE, "RX_PASSIVE"),
    (CAN_ERR_CRTL_TX_WARNING, "TX_WARN"),
    (CAN_ERR_CRTL_RX_WARNING, "RX_WARN"),
    (CAN_ERR_CRTL_ACTIVE, "ACTIVE"),
    (CAN_ERR_CRTL_RX_OVERFLOW, "RX_OVERFLOW"),
    (CAN_ERR_CRTL_TX_OVERFLOW, "TX_OVERFLOW"),
)

# 0x181 / 0x182 は、この PSoC がモータ状態として受信している TPDO。
# 0x381 / 0x382 は依頼どおり別に見る。出ていなければ列は空のまま。
AGE_IDS = (
    (0x090, "090"),
    (0x0A0, "0a0"),
    (0x0C0, "0c0"),
    (0x181, "181"),
    (0x182, "182"),
    (0x381, "381"),
    (0x382, "382"),
)
WATCH_IDS = tuple(can_id for can_id, _tag in AGE_IDS)

RE_STATE = re.compile(r"can state (\S+)")
RE_BITRATE = re.compile(r"\bbitrate\s+(\d+)")
RE_CLOCK = re.compile(r"\bclock\s+(\d+)")
RE_SAMP = re.compile(r"sample-point\s+([0-9.]+)")
RE_SJW = re.compile(r"\bsjw\s+(\d+)")
RE_RESTART = re.compile(r"restart-ms\s+(\d+)")
RE_QLEN = re.compile(r"\bqlen\s+(\d+)")
RE_CPU_TEMP = re.compile(r"temp=([0-9]+(?:\.[0-9]+)?)")
RE_STATS = re.compile(
    r"re-started\s+bus-errors\s+arbit-lost\s+error-warn\s+error-pass\s+bus-off\s+"
    r"(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)",
    re.S,
)
RE_RX = re.compile(
    r"RX:\s+bytes\s+packets\s+errors\s+dropped\s+\S+\s+\S+\s+"
    r"(\d+)\s+(\d+)\s+(\d+)\s+(\d+)",
    re.S,
)
RE_TX = re.compile(
    r"TX:\s+bytes\s+packets\s+errors\s+dropped\s+\S+\s+\S+\s+"
    r"(\d+)\s+(\d+)\s+(\d+)\s+(\d+)",
    re.S,
)
KERN_KEEP = re.compile(
    r"mcp251|\bcan\d|\bspi|overflow|bus-off|bus off"
    r"|oops|panic|internal error|unable to handle|kernel bug|\bbug:"
    r"|hung.task|blocked for more than"
    r"|out of memory|oom-kill|killed process"
    r"|watchdog|under-?voltage|thermal|throttl"
    r"|mmc\d|i/o error|ext4-fs|usb disconnect",
    re.I,
)
BLOCKED_TTY = {"serial0", "serial1", "ttyama0", "ttys0"}

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

CHANGE_KEYS = (
    "can_link",
    "can_state",
    "can_restarts",
    "can_bus_errors",
    "can_arbit_lost",
    "can_error_warn",
    "can_error_pass",
    "can_bus_off",
    "can_rx_err",
    "can_rx_drop",
    "can_tx_err",
    "can_tx_drop",
)
TIMING_KEYS = ("clock", "bitrate", "sample_point", "sjw", "restart_ms", "qlen")

UART_COLUMNS = [
    "pi_time",
    "psoc_time",
    "kind",
    "message",
    "sts1",
    "sts2",
    "sts3",
    "sts4",
    "m1_sw",
    "m1_state",
    "m1_remote",
    "m1_target",
    "m1_speed",
    "m1_limit",
    "m1_warn",
    "m1_trq",
    "m1_hwto",
    "m1_son",
    "m1_stop",
    "m2_sw",
    "m2_state",
    "m2_remote",
    "m2_target",
    "m2_speed",
    "m2_limit",
    "m2_warn",
    "m2_trq",
    "m2_hwto",
    "m2_son",
    "m2_stop",
    "nocmd_s",
    "corr_ms",
    "m1_silent_s",
    "m2_silent_s",
    "can_tec",
    "can_rec",
    "can_es",
    "can_class",
    "can_ack",
    "can_bit",
    "can_stuff",
    "can_form",
    "can_crc",
    "can_arb",
    "can_lost",
    "can_txfail",
    "can_stop_s",
]
KERN_COLUMNS = ["pi_time", "kern_time", "message"]
AGE_COLUMNS: list[str] = []
for _can_id, _tag in AGE_IDS:
    AGE_COLUMNS.append(f"last_{_tag}")
    AGE_COLUMNS.append(f"age_{_tag}_s")
POWER_COLUMNS = ["amp", "temp1", "temp2", "temp3"]
STATUS_COLUMNS = [
    "pi_time",
    "uptime_s",
    "cpu_temp_c",
    "cpu_pct",
    "load1",
    "mem_used_pct",
    "throttled",
    *POWER_COLUMNS,
    "can_link",
    "can_state",
    "can_restarts",
    "can_bus_errors",
    "can_arbit_lost",
    "can_error_warn",
    "can_error_pass",
    "can_bus_off",
    "can_rx",
    "can_rx_err",
    "can_rx_drop",
    "can_tx",
    "can_tx_err",
    "can_tx_drop",
    "clock",
    "bitrate",
    "sample_point",
    "sjw",
    "restart_ms",
    "qlen",
    *AGE_COLUMNS,
]
CAN_COLUMNS = [
    "pi_time",
    "kind",
    "can_link",
    "can_state",
    "can_restarts",
    "can_bus_errors",
    "can_arbit_lost",
    "can_error_warn",
    "can_error_pass",
    "can_bus_off",
    "can_rx_err",
    "can_rx_drop",
    "can_tx_err",
    "can_tx_drop",
    "clock",
    "bitrate",
    "sample_point",
    "sjw",
    "restart_ms",
    "qlen",
    "err_cause",
    "err_class",
    "err_prot",
    "err_loc",
    "err_ctrl",
    "err_data",
    "app_unit",
    "app_active",
    "app_sub",
    "app_since",
    *POWER_COLUMNS,
    *AGE_COLUMNS,
    "detail",
]
CAN_ERR_COLUMNS = (
    "err_cause",
    "err_class",
    "err_prot",
    "err_loc",
    "err_ctrl",
    "err_data",
)
CAN_APP_COLUMNS = (
    "app_unit",
    "app_active",
    "app_sub",
    "app_since",
)
CAN_ABSENT_COLUMNS = CAN_ERR_COLUMNS + CAN_APP_COLUMNS + ("detail",)


def pi_now() -> str:
    """現地時刻。オフセット付き。ミリ秒まで。"""
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def clock_text(raw: str) -> str:
    matched = CLOCK_FACE.match(raw)
    if matched is not None:
        return f"{matched.group('h')}:{matched.group('m')}:{matched.group('s')}"
    if raw in ("--/--/--", "--:--:--"):
        return "--:--:--"
    return raw


def split_line(text: str) -> tuple[str, str]:
    matched = PSOC_TIME.match(text.strip())
    if matched is None:
        return "", text.strip()
    rest = matched.group("rest")
    if rest.startswith(","):
        rest = rest[1:]
    return clock_text(matched.group("psoc")), rest


def cia_state(sw: int) -> str:
    if (sw & 0x004F) == 0x0000:
        return "準備前"
    if (sw & 0x004F) == 0x0040:
        return "励磁不可"
    if (sw & 0x006F) == 0x0021:
        return "準備完了"
    if (sw & 0x006F) == 0x0023:
        return "スイッチオン"
    if (sw & 0x006F) == 0x0027:
        return "運転可"
    if (sw & 0x006F) == 0x0007:
        return "急停止"
    if (sw & 0x004F) == 0x000F:
        return "異常処理"
    if (sw & 0x004F) == 0x0008:
        return "異常"
    return "不明"


def din_text(din: str, bit: int) -> str:
    if din == "":
        return "不明"
    value = int(din, 16)
    return "入" if (value & (1 << bit)) != 0 else "切"


def motor_fields(prefix: str, sw_text: str, trq_text: str, din: str) -> dict[str, str]:
    sw = int(sw_text, 16) if sw_text else 0
    return {
        f"{prefix}_sw": sw_text,
        f"{prefix}_state": cia_state(sw) if sw_text else "",
        f"{prefix}_remote": ("遠隔" if (sw & 0x0200) != 0 else "ローカル") if sw_text else "",
        f"{prefix}_target": ("到達" if (sw & 0x0400) != 0 else "未到達") if sw_text else "",
        f"{prefix}_speed": ("速度0" if (sw & 0x1000) != 0 else "回転") if sw_text else "",
        f"{prefix}_limit": "内部制限" if sw_text and (sw & 0x0800) != 0 else ("-" if sw_text else ""),
        f"{prefix}_warn": "警告" if sw_text and (sw & 0x0080) != 0 else ("-" if sw_text else ""),
        f"{prefix}_trq": trq_text if trq_text != "" else "不明",
        f"{prefix}_hwto": din_text(din, 3),
        f"{prefix}_son": din_text(din, 16),
        f"{prefix}_stop": din_text(din, 21),
    }


def blank_uart() -> dict[str, str]:
    return {name: "" for name in UART_COLUMNS}


def split_dump_rest(rest: str) -> tuple[str, dict[str, str]]:
    matched = DUMP_STS.match(rest)
    if matched is None:
        return rest, {}
    return matched.group("text") or "", {
        "sts1": matched.group("s1").lower(),
        "sts2": matched.group("s2").lower(),
        "sts3": matched.group("s3").lower(),
        "sts4": matched.group("s4").lower(),
    }


def uart_note(message: str) -> dict[str, str]:
    row = blank_uart()
    row["pi_time"] = pi_now()
    row["kind"] = "note"
    row["message"] = message
    return row


def uart_from_line(psoc_time: str, rest: str, kind_hint: str = "") -> dict[str, str]:
    row = blank_uart()
    row["pi_time"] = pi_now()
    row["psoc_time"] = psoc_time
    row["message"] = rest
    parts = rest.split(",")
    tag = parts[0] if parts else ""
    if tag == "M" and len(parts) >= 11:
        row["kind"] = "M"
        row.update(motor_fields("m1", parts[1], parts[2], parts[3]))
        row.update(motor_fields("m2", parts[4], parts[5], parts[6]))
        row["nocmd_s"] = parts[7]
        row["corr_ms"] = parts[8]
        row["m1_silent_s"] = parts[9]
        row["m2_silent_s"] = parts[10]
        for key in UART_COLUMNS[UART_COLUMNS.index("can_tec"):]:
            row[key] = "-"
    elif tag == "CANOFF" and len(parts) >= 4:
        row["kind"] = "CANOFF"
        row["can_tec"] = parts[1]
        row["can_rec"] = parts[2]
        row["can_es"] = parts[3]
    elif tag == "CANPASV" and len(parts) >= 2:
        row["kind"] = "CANPASV"
        row["can_class"] = parts[1]
    elif tag == "CANERR" and len(parts) >= 8:
        row["kind"] = "CANERR"
        keys = ("can_ack", "can_bit", "can_stuff", "can_form", "can_crc", "can_arb", "can_lost")
        for key, value in zip(keys, parts[1:8]):
            row[key] = value
        if len(parts) > 8:
            row["can_txfail"] = " ".join(parts[8:])
    elif tag == "CANSTOP" and len(parts) >= 2:
        row["kind"] = "CANSTOP"
        row["can_stop_s"] = parts[1]
    else:
        row["kind"] = kind_hint or "event"
    return row


def names_of(value: int, bits: tuple[tuple[int, str], ...]) -> list[str]:
    return [label for bit, label in bits if value & bit]


def err_cause(prot: list[str], loc: str, classes: list[str], ctrl: list[str]) -> str:
    parts = list(prot)
    if loc in ("CRC_SEQ", "CRC_DEL") and "CRC" not in parts:
        parts.append("CRC")
    if parts:
        return "+".join(parts)
    if "ACK" in classes or loc in ("ACK", "ACK_DEL"):
        return "ACK"
    if ctrl:
        return "+".join(ctrl)
    if "BUSOFF" in classes:
        return "bus-off"
    if classes:
        return "+".join(classes)
    return ""


def decode_error(can_id: int, data: bytes) -> dict[str, str]:
    err = can_id & CAN_ERR_MASK
    classes = names_of(err, CLASS_BITS)
    prot_byte = data[2] if len(data) > 2 else 0
    loc_byte = data[3] if len(data) > 3 else 0
    ctrl_byte = data[1] if len(data) > 1 else 0
    prot = names_of(prot_byte, PROT_BITS) if err & CAN_ERR_PROT else []
    loc = PROT_LOC.get(loc_byte, f"0x{loc_byte:02x}") if err & CAN_ERR_PROT or loc_byte else ""
    if err & CAN_ERR_PROT and loc_byte == 0:
        loc = "UNSPEC"
    ctrl = names_of(ctrl_byte, CTRL_BITS) if err & CAN_ERR_CRTL else []
    return {
        "err_cause": err_cause(prot, loc, classes, ctrl),
        "err_class": "+".join(classes),
        "err_prot": "+".join(prot),
        "err_loc": loc,
        "err_ctrl": "+".join(ctrl),
        "err_data": data.hex(),
    }


def change_key(row: dict[str, str]) -> tuple[str, ...]:
    return tuple(row.get(key, "") for key in CHANGE_KEYS)


def timing_key(row: dict[str, str]) -> tuple[str, ...]:
    return tuple(row.get(key, "") for key in TIMING_KEYS)


def timing_detail(row: dict[str, str]) -> str:
    note = "PSoCは16tq サンプル0.875 SJW2"
    restart = row.get("restart_ms", "")
    if restart == "0":
        note += "。restart-ms 0 は bus-off のあと自動復帰しない"
    sample = row.get("sample_point", "")
    sjw = row.get("sjw", "")
    if sample and sjw:
        try:
            same = abs(float(sample) - 0.875) < 0.001 and int(sjw) == 2
        except ValueError:
            same = False
        note += "。サンプルとSJWはPSoCと同じ" if same else "。サンプルまたはSJWがPSoCと違う"
    return note


def port_block_reason(path: str) -> str:
    base = path.replace("\\", "/").rstrip("/").split("/")[-1].lower()
    if base in BLOCKED_TTY or base.startswith("ttyama"):
        return "バッテリ監視のシリアルなので開かない"
    if path and os.path.exists(path):
        real = os.path.basename(os.path.realpath(path)).lower()
        if real in BLOCKED_TTY or real.startswith("ttyama"):
            return "バッテリ監視のシリアルなので開かない"
    return ""


def find_port() -> tuple[str, str]:
    """(path, error)。by-id の CP210x だけ。ttyUSB や serial0 には落とさない。"""
    by_id = Path("/dev/serial/by-id")
    hits: list[Path] = []
    if by_id.is_dir():
        for path in sorted(by_id.iterdir()):
            name = path.name.lower()
            if "cp210" in name or "silicon_labs" in name or "siliconlabs" in name:
                hits.append(path)
    if len(hits) == 1:
        chosen = str(hits[0])
        reason = port_block_reason(chosen)
        if reason:
            return "", reason
        return chosen, ""
    if len(hits) > 1:
        names = ", ".join(str(path) for path in hits)
        return "", "CP210x が複数あります。--port で指定してください " + names
    return "", "CP210x の by-id が見つかりません"


def open_serial(path: str) -> int:
    import termios

    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


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


def throttle_text(value: int) -> str:
    names = [label for bit, label in THROTTLE_BITS if value & bit]
    return " ".join(names) if names else "なし"


def read_text(path: str) -> str:
    try:
        return Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""


def read_uptime() -> str:
    text = read_text("/proc/uptime")
    return text.split()[0] if text else ""


def parse_cpu_temp(command_text: str, sysfs_text: str) -> str:
    """vcgencmd measure_temp の値。コマンドが無ければ sysfs の同じ温度。"""
    matched = RE_CPU_TEMP.search(command_text)
    if matched is not None:
        return f"{float(matched.group(1)):.1f}"
    try:
        return f"{int(sysfs_text.strip()) / 1000.0:.1f}"
    except ValueError:
        return ""


def read_cpu_temp() -> str:
    command = run_cmd(["vcgencmd", "measure_temp"], 0.4)
    sysfs = read_text("/sys/class/thermal/thermal_zone0/temp")
    return parse_cpu_temp(command, sysfs)


def run_cmd(args: list[str], timeout: float) -> str:
    try:
        proc = subprocess.run(
            args,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return ""
    if proc.returncode != 0 and not proc.stdout:
        return ""
    return (proc.stdout or "") + (proc.stderr or "")


def read_load1() -> str:
    text = read_text("/proc/loadavg")
    return text.split()[0] if text else ""


def read_mem_used_pct() -> str:
    text = read_text("/proc/meminfo")
    if not text:
        return ""
    info: dict[str, int] = {}
    for line in text.splitlines():
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
    text = read_text("/proc/stat")
    if not text:
        return None
    try:
        nums = [int(item) for item in text.splitlines()[0].split()[1:]]
    except ValueError:
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


def read_throttled() -> str:
    text = run_cmd(["vcgencmd", "get_throttled"], 0.4).strip()
    if "=" not in text:
        return ""
    raw = text.split("=")[-1].strip()
    try:
        value = int(raw, 0)
    except ValueError:
        return ""
    return f"0x{value:x} {throttle_text(value)}"


def link_text(iface: str) -> str:
    return run_cmd(["ip", "-details", "-statistics", "link", "show", iface], 0.25)


def parse_can(text: str) -> dict[str, str]:
    row = {
        "can_link": "",
        "can_state": "",
        "can_restarts": "",
        "can_bus_errors": "",
        "can_arbit_lost": "",
        "can_error_warn": "",
        "can_error_pass": "",
        "can_bus_off": "",
        "can_rx": "",
        "can_rx_err": "",
        "can_rx_drop": "",
        "can_tx": "",
        "can_tx_err": "",
        "can_tx_drop": "",
        "clock": "",
        "bitrate": "",
        "sample_point": "",
        "sjw": "",
        "restart_ms": "",
        "qlen": "",
    }
    if not text:
        return row
    if "does not exist" in text or "Cannot find device" in text:
        row["can_link"] = "なし"
        return row
    first = text.split("\n", 1)[0]
    if " state UP" in first:
        row["can_link"] = "UP"
    elif " state DOWN" in first:
        row["can_link"] = "DOWN"
    found = RE_STATE.search(text)
    if found:
        row["can_state"] = found.group(1)
    for regex, key in (
        (RE_BITRATE, "bitrate"),
        (RE_CLOCK, "clock"),
        (RE_SAMP, "sample_point"),
        (RE_SJW, "sjw"),
        (RE_RESTART, "restart_ms"),
        (RE_QLEN, "qlen"),
    ):
        matched = regex.search(text)
        if matched:
            row[key] = matched.group(1)
    stats = RE_STATS.search(text)
    if stats:
        row["can_restarts"] = stats.group(1)
        row["can_bus_errors"] = stats.group(2)
        row["can_arbit_lost"] = stats.group(3)
        row["can_error_warn"] = stats.group(4)
        row["can_error_pass"] = stats.group(5)
        row["can_bus_off"] = stats.group(6)
    rx = RE_RX.search(text)
    if rx:
        row["can_rx"] = rx.group(2)
        row["can_rx_err"] = rx.group(3)
        row["can_rx_drop"] = rx.group(4)
    tx = RE_TX.search(text)
    if tx:
        row["can_tx"] = tx.group(2)
        row["can_tx_err"] = tx.group(3)
        row["can_tx_drop"] = tx.group(4)
    return row


def set_can_opt(sock: socket.socket, opt: int, value: bytes) -> None:
    try:
        sock.setsockopt(SOL_CAN_RAW, opt, value)
    except OSError:
        sock.setsockopt(100, opt, value)


def open_can_socket(iface: str, filters: bytes, err_mask: int | None) -> socket.socket | None:
    try:
        sock = socket.socket(AF_CAN, socket.SOCK_RAW, CAN_RAW)
        sock.setblocking(False)
        if err_mask is not None:
            set_can_opt(sock, CAN_RAW_ERR_FILTER, struct.pack("=I", err_mask))
        set_can_opt(sock, CAN_RAW_FILTER, filters)
        sock.bind((iface,))
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
    except OSError:
        return None
    return sock


def open_error_socket(iface: str) -> socket.socket | None:
    filt = struct.pack("=II", CAN_ERR_FLAG, CAN_ERR_FLAG)
    return open_can_socket(iface, filt, CAN_ERR_MASK)


def open_id_socket(iface: str) -> socket.socket | None:
    filt = b"".join(struct.pack("=II", can_id, CAN_SFF_MASK) for can_id in WATCH_IDS)
    return open_can_socket(iface, filt, None)


def unpack_frame(raw: bytes) -> tuple[int, int, bytes] | None:
    if len(raw) < CAN_FRAME_SIZE:
        return None
    can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw[:CAN_FRAME_SIZE])
    return can_id, dlc, data[: min(dlc, 8)]


def drain_frames(sock: socket.socket | None, limit: int = 500) -> list[tuple[int, int, bytes]]:
    if sock is None:
        return []
    frames: list[tuple[int, int, bytes]] = []
    while len(frames) < limit:
        try:
            raw = sock.recv(CAN_FRAME_SIZE)
        except (BlockingIOError, InterruptedError):
            break
        except OSError:
            break
        unpacked = unpack_frame(raw)
        if unpacked is None:
            continue
        can_id, dlc, data = unpacked
        if can_id & 0x40000000:
            continue
        frames.append((can_id, dlc, data))
    return frames


def u16be(data: bytes, offset: int) -> int:
    return (data[offset] << 8) | data[offset + 1]


class DayFile:
    def __init__(
        self,
        directory: Path,
        prefix: str,
        columns: list[str],
        keep_files: int,
        stamp: str,
    ) -> None:
        self.directory = directory
        self.prefix = prefix
        self.columns = columns
        self.keep_files = keep_files
        self.stamp = stamp
        self.handle = None
        self.writer: csv.DictWriter | None = None
        self.path: Path | None = None
        self.dirty = False
        self.last_sync = 0.0

    def ensure(self) -> None:
        if self.handle is not None:
            return
        self.directory.mkdir(parents=True, exist_ok=True)
        path = self._compatible_path(self.directory / f"{self.prefix}_{self.stamp}.csv")
        new = (not path.exists()) or path.stat().st_size == 0
        self.handle = path.open("a", encoding="utf-8", newline="")
        self.writer = csv.DictWriter(
            self.handle,
            fieldnames=self.columns,
            restval="",
            extrasaction="ignore",
        )
        if new:
            self.writer.writeheader()
            self.handle.flush()
            os.fsync(self.handle.fileno())
        self.path = path
        prune_count(self.directory, self.prefix, self.keep_files, path)

    def _compatible_path(self, path: Path) -> Path:
        if (not path.exists()) or path.stat().st_size == 0:
            return path
        with path.open(encoding="utf-8") as handle:
            header = handle.readline().rstrip("\r\n")
        if header == ",".join(self.columns):
            return path
        return path.with_name(f"{path.stem}_v2{path.suffix}")

    def write(self, row: dict[str, str], sync: bool = True) -> None:
        self.ensure()
        assert self.writer is not None and self.handle is not None
        self.writer.writerow(row)
        self.handle.flush()
        if sync:
            os.fsync(self.handle.fileno())
            self.dirty = False
            self.last_sync = time.monotonic()
        else:
            self.dirty = True

    def sync_pending(self, now: float, interval: float = 0.2) -> None:
        if not self.dirty or self.handle is None:
            return
        if (now - self.last_sync) < interval:
            return
        os.fsync(self.handle.fileno())
        self.dirty = False
        self.last_sync = now

    def close(self) -> None:
        if self.handle is None:
            return
        self.handle.flush()
        os.fsync(self.handle.fileno())
        self.handle.close()
        self.handle = None
        self.writer = None


def prune_count(directory: Path, prefix: str, keep: int, current: Path) -> None:
    """同じ接頭辞の CSV が keep 個を超えたら、名前の古いものから消す。"""
    if keep < 1 or not directory.is_dir():
        return
    try:
        current_resolved = current.resolve()
    except OSError:
        current_resolved = current
    others: list[Path] = []
    for path in directory.glob(f"{prefix}_*.csv"):
        if not path.is_file():
            continue
        try:
            if path.resolve() == current_resolved:
                continue
        except OSError:
            continue
        others.append(path)
    overflow = len(others) + 1 - keep
    if overflow <= 0:
        return
    others.sort(key=lambda path: path.name)
    for path in others[:overflow]:
        try:
            path.unlink()
        except OSError:
            pass


def mark_clear(row: dict[str, str], keys: tuple[str, ...]) -> None:
    """値が無い列を - にする。"""
    for key in keys:
        if row.get(key, "") == "":
            row[key] = "-"


def blank_can() -> dict[str, str]:
    return {key: "" for key in CAN_COLUMNS}


def age_fields(seen: dict[int, tuple[float, str]], now: float) -> dict[str, str]:
    row: dict[str, str] = {}
    for can_id, tag in AGE_IDS:
        hit = seen.get(can_id)
        if hit is None:
            row[f"last_{tag}"] = "-"
            row[f"age_{tag}_s"] = "-"
        else:
            mono, stamp = hit
            row[f"last_{tag}"] = stamp
            row[f"age_{tag}_s"] = f"{now - mono:.1f}"
    return row


def kernel_keep(line: str) -> bool:
    return KERN_KEEP.search(line) is not None


def previous_kernel_lines() -> list[str]:
    """前回起動の末尾から、落ちた理由と CAN の行を取る。"""
    text = run_cmd(
        ["journalctl", "-b", "-1", "-k", "-n", "400", "-o", "short-iso", "--no-pager"],
        3.0,
    )
    kept: list[str] = []
    for line in text.splitlines():
        line = line.strip()
        if line and kernel_keep(line) and not line.startswith("-- "):
            kept.append(line)
    return kept


def split_kern(line: str) -> tuple[str, str]:
    head, _, tail = line.partition(" ")
    if "T" in head and any(ch.isdigit() for ch in head):
        return head, tail
    return "", line


class KernelFollow:
    def __init__(self) -> None:
        self.proc: subprocess.Popen | None = None
        self.buf = b""
        self.next_try = 0.0

    def ensure(self, now: float) -> str:
        if self.proc is not None and self.proc.poll() is None:
            return ""
        if now < self.next_try:
            return ""
        self.next_try = now + 5.0
        try:
            self.proc = subprocess.Popen(
                ["journalctl", "-k", "-f", "-n", "20", "-o", "short-iso"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=0,
            )
        except OSError as exc:
            self.proc = None
            return f"journalctl を開始できません: {exc}"
        if self.proc.stdout is None:
            self.proc = None
            return "journalctl の出力がありません"
        os.set_blocking(self.proc.stdout.fileno(), False)
        return ""

    def fileno(self) -> int | None:
        if self.proc is None or self.proc.stdout is None or self.proc.poll() is not None:
            return None
        return self.proc.stdout.fileno()

    def read_lines(self) -> list[str]:
        fd = self.fileno()
        if fd is None:
            return []
        try:
            chunk = os.read(fd, 4096)
        except (BlockingIOError, InterruptedError, OSError):
            return []
        if not chunk:
            return []
        self.buf += chunk
        lines: list[str] = []
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            text = raw.decode("utf-8", errors="replace").strip()
            if text and kernel_keep(text):
                lines.append(text)
        return lines

    def close(self) -> None:
        if self.proc is None:
            return
        self.proc.terminate()
        try:
            self.proc.wait(timeout=1)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        self.proc = None


def read_app(unit: str) -> dict[str, str] | None:
    text = run_cmd(
        [
            "systemctl",
            "show",
            unit,
            "-p",
            "LoadState",
            "-p",
            "ActiveState",
            "-p",
            "SubState",
            "-p",
            "ActiveEnterTimestamp",
            "-p",
            "InactiveEnterTimestamp",
            "--no-pager",
        ],
        0.4,
    )
    if not text:
        return None
    fields: dict[str, str] = {}
    for line in text.splitlines():
        key, _, value = line.partition("=")
        fields[key] = value.strip()
    if fields.get("LoadState") == "not-found":
        return {"app_unit": unit, "app_active": "不明", "app_sub": "", "app_since": ""}
    active = fields.get("ActiveState", "")
    since = fields.get("ActiveEnterTimestamp", "") if active == "active" else fields.get(
        "InactiveEnterTimestamp", ""
    )
    return {
        "app_unit": unit,
        "app_active": active,
        "app_sub": fields.get("SubState", ""),
        "app_since": since,
    }


def send_log_request(fd: int) -> None:
    for byte in b"LOG\n":
        try:
            os.write(fd, bytes([byte]))
        except OSError:
            return
        time.sleep(0.008)


class UartLink:
    def __init__(self, prefer: str) -> None:
        self.prefer = prefer
        self.fd: int | None = None
        self.path = ""
        self.pending = bytearray()
        self.phase = "down"
        self.next_try = 0.0
        self.next_send = 0.0
        self.begin_deadline = 0.0
        self.end_deadline = 0.0
        self.stored = 0
        self.dump_since_sync = 0
        self.last_fail = ""
        self.next_fail_log = 0.0

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
        self.fd = None
        self.phase = "down"
        self.pending.clear()

    def fail(self, book: DayFile, reason: str, now: float) -> None:
        if reason == self.last_fail and now < self.next_fail_log:
            return
        self.last_fail = reason
        self.next_fail_log = now + 60.0
        book.write(uart_note("SERIAL " + reason))

    def try_open(self, book: DayFile, now: float) -> None:
        if self.fd is not None or now < self.next_try:
            return
        self.next_try = now + 2.0
        path = self.prefer
        if not path:
            path, reason = find_port()
            if not path:
                self.fail(book, reason, now)
                return
        blocked = port_block_reason(path)
        if blocked:
            self.fail(book, blocked + " " + path, now)
            return
        try:
            self.fd = open_serial(path)
        except OSError as exc:
            self.fd = None
            self.fail(book, f"オープン失敗 {path}: {exc}", now)
            return
        self.path = path
        self.phase = "hunt"
        self.stored = 0
        self.begin_deadline = time.monotonic() + 15.0
        self.next_send = 0.0
        self.last_fail = ""
        book.write(uart_note(f"SERIAL 接続 {path}"))

    def pump(self, book: DayFile, now: float) -> None:
        if self.fd is None:
            return
        if self.phase == "hunt" and now >= self.next_send:
            send_log_request(self.fd)
            self.next_send = now + 5.0
        if self.phase == "hunt" and now >= self.begin_deadline:
            book.write(uart_note("保存ログは取れませんでした。このあとの行だけ記録します"))
            self.phase = "live"
        if self.phase == "dump" and now >= self.end_deadline:
            book.write(
                uart_note(f"保存ログの終わりが来ませんでした。取れた {self.stored} 件"),
                sync=True,
            )
            self.phase = "live"

    def on_bytes(self, chunk: bytes, book: DayFile) -> None:
        self.pending.extend(chunk)
        for text in take_lines(self.pending):
            self.on_line(text, book)

    def on_line(self, text: str, book: DayFile) -> None:
        now = time.monotonic()
        if text == "LOGBEGIN":
            self.phase = "dump"
            self.end_deadline = now + 10.0
            self.stored = 0
            book.write(uart_note("保存ログ開始"), sync=False)
            return
        if text == "LOGEND":
            book.write(uart_note(f"保存ログ終了 {self.stored} 件"), sync=True)
            self.phase = "live"
            return
        matched = DUMP_LINE.match(text)
        if self.phase == "dump" and matched is not None:
            text, sts = split_dump_rest(matched.group("rest"))
            row = uart_from_line(matched.group("psoc"), text, "dump")
            row.update(sts)
            book.write(row, sync=False)
            self.stored += 1
            self.dump_since_sync += 1
            self.end_deadline = now + 10.0
            if self.dump_since_sync >= 20:
                book.sync_pending(now, 0.0)
                self.dump_since_sync = 0
            return
        psoc_time, message = split_line(text)
        book.write(uart_from_line(psoc_time, message))
        print(text, flush=True)


def self_test() -> int:
    failed = 0

    def check(cond: bool, label: str) -> None:
        nonlocal failed
        if not cond:
            failed += 1
            print("FAIL", label)

    stamp = pi_now()
    check(bool(re.search(r"T\d{2}:\d{2}:\d{2}\.\d{3}[+-]\d{2}:\d{2}$", stamp)), "pi_now " + stamp)
    check(clock_text("11/12/45") == "11:12:45", "slash clock")
    check(split_line("11/12/45 M1 運転可") == ("11:12:45", "M1 運転可"), "split")
    check(split_line("11:12:45,START 1") == ("11:12:45", "START 1"), "split comma")
    motor = uart_from_line(
        "17:21:00",
        "M,2f27,-17,00000008,0040,,,101,0,1,2",
    )
    check(motor["kind"] == "M", "motor kind")
    check(motor["m1_state"] == "運転可" and motor["m1_remote"] == "遠隔", "m1 state")
    check(motor["m1_target"] == "到達" and motor["m1_speed"] == "回転", "m1 bits")
    check(motor["m1_limit"] == "内部制限" and motor["m1_trq"] == "-17", "m1 trq")
    check(motor["m1_warn"] == "-" and motor["m2_limit"] == "-" and motor["m2_warn"] == "-", "clear flags")
    check(motor["can_tec"] == "-" and motor["can_stop_s"] == "-", "motor can dash")
    unseen = age_fields({}, 0.0)
    check(unseen["last_090"] == "-" and unseen["age_181_s"] == "-", "unseen age")
    seen_age = age_fields({0x181: (0.0, "t")}, 1.5)
    check(seen_age["last_181"] == "t" and seen_age["age_181_s"] == "1.5", "seen age")
    clear_row = {"err_cause": "", "detail": "残す"}
    mark_clear(clear_row, ("err_cause", "detail"))
    check(clear_row["err_cause"] == "-" and clear_row["detail"] == "残す", "mark clear")
    check(motor["m1_hwto"] == "入" and motor["m1_son"] == "切" and motor["m1_stop"] == "切", "m1 din")
    check(motor["m2_state"] == "励磁不可" and motor["m2_trq"] == "不明", "m2 missing trq")
    check(motor["m2_hwto"] == "不明" and motor["nocmd_s"] == "101", "m2 din empty")
    check(motor["m1_silent_s"] == "1" and motor["m2_silent_s"] == "2", "silence")
    can = uart_from_line("17:21:00", "CANERR,0,0,0,0,0,2,0,181=3")
    check(can["can_arb"] == "2" and can["can_txfail"] == "181=3" and can["can_tec"] == "", "canerr")
    text, sts = split_dump_rest("01,20,03,0a START 1")
    check(text == "START 1" and sts["sts2"] == "20" and sts["sts3"] == "03", "dump sts")
    check(split_dump_rest("RASP RS")[0] == "RASP RS", "dump without sts")
    check(parse_cpu_temp("temp=48.2'C\n", "") == "48.2", "vcgencmd temp")
    check(parse_cpu_temp("", "51234\n") == "51.2", "sysfs temp")
    check(port_block_reason("/dev/serial0") != "", "block serial0")
    check(port_block_reason("/dev/serial/by-id/usb-Silicon_Labs_CP2102") == "", "allow cp210")

    ack = decode_error(CAN_ERR_FLAG | CAN_ERR_ACK, bytes([0, 0, 0, 0x19, 0, 0, 1, 2]))
    check(ack["err_cause"] == "ACK" and ack["err_loc"] == "ACK", "ack " + ack["err_cause"])
    bit0 = decode_error(CAN_ERR_FLAG | CAN_ERR_PROT, bytes([0, 0, 0x08, 0x08, 0, 0, 9, 8]))
    check(bit0["err_cause"] == "bit0+CRC" and bit0["err_loc"] == "CRC_SEQ", "bit0+crc " + bit0["err_cause"])
    stuff = decode_error(CAN_ERR_FLAG | CAN_ERR_PROT, bytes([0, 0, 0x04, 0x0A, 0, 0, 3, 4]))
    check(stuff["err_cause"] == "stuff" and stuff["err_loc"] == "DATA", "stuff")
    form = decode_error(CAN_ERR_FLAG | CAN_ERR_PROT, bytes([0, 0, 0x02, 0x18, 0, 0, 5, 6]))
    check(form["err_cause"] == "form+CRC", "form+crc " + form["err_cause"])

    same = {"can_link": "UP", "can_state": "ERROR-ACTIVE", "can_rx": "1"}
    other = dict(same)
    other["can_rx"] = "99"
    changed = dict(same)
    changed["can_state"] = "BUS-OFF"
    check(change_key(same) == change_key(other), "rx is not a change")
    check(change_key(same) != change_key(changed), "state is a change")
    tx_err = dict(same)
    tx_err["can_tx_err"] = "3"
    check(change_key(same) != change_key(tx_err), "tx err is a change")
    check(kernel_keep("Kernel panic - not syncing"), "keep panic")
    check(kernel_keep("hwmon: Under-voltage detected!"), "keep undervoltage")
    check(kernel_keep("mcp251x can0: bus-off"), "keep can")
    check(not kernel_keep("systemd[1]: Started foo"), "drop other kernel")
    zero = {"sample_point": "0.875", "sjw": "2", "restart_ms": "0"}
    check("自動復帰しない" in timing_detail(zero), "restart 0")
    check("同じ" in timing_detail({"sample_point": "0.875", "sjw": "2", "restart_ms": "100"}), "match")
    with tempfile.TemporaryDirectory() as tmp:
        folder = Path(tmp)
        for n in range(4):
            (folder / f"psoc_uart_2026010{n}_000000.csv").write_text("x", encoding="utf-8")
        current = folder / "psoc_uart_20260103_000000.csv"
        prune_count(folder, "psoc_uart", 3, current)
        left = sorted(path.name for path in folder.glob("psoc_uart_*.csv"))
        check(
            left == [
                "psoc_uart_20260101_000000.csv",
                "psoc_uart_20260102_000000.csv",
                "psoc_uart_20260103_000000.csv",
            ],
            "keep newest files " + ",".join(left),
        )

    print("self-test", "ok" if failed == 0 else f"{failed} failed")
    return 1 if failed else 0


def main() -> int:
    if "--self-test" in sys.argv:
        return self_test()

    parser = argparse.ArgumentParser(description="PSoC UART と CAN の変化を CSV に記録する")
    parser.add_argument("--port", default="", help="by-id の固定名。省略時は CP210x を探す")
    parser.add_argument("--dir", default="log")
    parser.add_argument("--interval", type=float, default=10.0, help="状態の記録間隔（秒）")
    parser.add_argument("--can-ms", type=float, default=150.0, help="CAN を見る間隔（ミリ秒）")
    parser.add_argument("--iface", default="can0")
    parser.add_argument(
        "--app-unit",
        default=os.environ.get("PSOC_PI_LOG_APP_UNIT", ""),
        help="起動と終了を残す systemd unit。未指定なら残さない",
    )
    parser.add_argument("--keep-files", type=int, default=100)
    args = parser.parse_args()
    if args.interval < 1.0:
        parser.error("--interval は 1 以上")
    if not 50.0 <= args.can_ms <= 500.0:
        parser.error("--can-ms は 50 から 500")
    if args.keep_files < 1:
        parser.error("--keep-files は 1 以上")

    root = Path(args.dir)
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    uart_book = DayFile(root / "uart", "psoc_uart", UART_COLUMNS, args.keep_files, stamp)
    status_book = DayFile(root / "status", "pi_status", STATUS_COLUMNS, args.keep_files, stamp)
    can_book = DayFile(root / "can", "can_event", CAN_COLUMNS, args.keep_files, stamp)
    kern_book = DayFile(root / "kernel", "kern", KERN_COLUMNS, args.keep_files, stamp)
    uart_book.ensure()
    can_book.ensure()
    print(f"UART: {uart_book.path}", flush=True)
    print(f"状態: {status_book.directory}", flush=True)
    print(f"CAN: {can_book.path}", flush=True)
    print(
        f"状態は {args.interval:.0f} 秒、CAN は {args.can_ms:.0f} ms。"
        " シリアルは by-id。止めるときは Ctrl+C",
        flush=True,
    )
    uart_book.write(uart_note("記録開始" + (f" port={args.port}" if args.port else "")))
    previous = previous_kernel_lines()
    if previous:
        kern_book.write({"pi_time": pi_now(), "kern_time": "", "message": "前回起動の該当行"})
        for line in previous:
            kern_time, message = split_kern(line)
            kern_book.write(
                {"pi_time": pi_now(), "kern_time": kern_time, "message": message},
                sync=False,
            )
        kern_book.sync_pending(time.monotonic(), 0.0)
    else:
        kern_book.write(
            {"pi_time": pi_now(), "kern_time": "", "message": "前回起動に該当行はありません"}
        )

    link = UartLink(args.port)
    kernel = KernelFollow()
    err_sock: socket.socket | None = None
    id_sock: socket.socket | None = None
    next_sock = 0.0
    sock_failed = False
    seen: dict[int, tuple[float, str]] = {}
    power = {"amp": "", "temp1": "", "temp2": "", "temp3": ""}
    can_now: dict[str, str] = parse_can("")
    prev_change: tuple[str, ...] | None = None
    prev_timing: tuple[str, ...] | None = None
    prev_app: dict[str, str] | None = None
    cpu = read_cpu_jiffies()
    next_can = 0.0
    next_status = 0.0
    next_app = 0.0
    can_interval = args.can_ms / 1000.0

    def power_now() -> dict[str, str]:
        return {key: value if value != "" else "-" for key, value in power.items()}

    def context(kind: str) -> dict[str, str]:
        row = blank_can()
        row["pi_time"] = pi_now()
        row["kind"] = kind
        row.update({key: can_now.get(key, "") for key in CHANGE_KEYS})
        row.update({key: can_now.get(key, "") for key in TIMING_KEYS})
        row.update(power_now())
        row.update(age_fields(seen, time.monotonic()))
        mark_clear(row, CAN_ABSENT_COLUMNS)
        return row

    def pull_can() -> None:
        if err_sock is not None:
            for can_id, _dlc, data in drain_frames(err_sock):
                if not (can_id & CAN_ERR_FLAG):
                    continue
                decoded = decode_error(can_id, data)
                row = context("error")
                row.update(decoded)
                mark_clear(row, CAN_ERR_COLUMNS)
                can_book.write(row, sync=False)
        if id_sock is not None:
            for can_id, dlc, data in drain_frames(id_sock):
                if can_id & CAN_ERR_FLAG:
                    continue
                sid = can_id & CAN_SFF_MASK
                if sid in {item[0] for item in AGE_IDS}:
                    seen[sid] = (time.monotonic(), pi_now())
                if sid == 0x090 and dlc >= 8:
                    power["amp"] = str(u16be(data, 0))
                    power["temp1"] = str(u16be(data, 2))
                    power["temp2"] = str(u16be(data, 4))
                    power["temp3"] = str(u16be(data, 6))

    def request_stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    try:
        while True:
            now = time.monotonic()
            pull_can()
            link.try_open(uart_book, now)
            link.pump(uart_book, now)
            kern_note = kernel.ensure(now)
            if kern_note:
                kern_book.write({"pi_time": pi_now(), "kern_time": "", "message": kern_note})
                kern_note = ""
            if (err_sock is None or id_sock is None) and now >= next_sock:
                next_sock = now + 5.0
                if err_sock is None:
                    err_sock = open_error_socket(args.iface)
                if id_sock is None:
                    id_sock = open_id_socket(args.iface)
                opened = err_sock is not None and id_sock is not None
                if not opened and not sock_failed:
                    row = context("state")
                    row["detail"] = "CAN ソケットを開けません。ip の状態だけ残します"
                    can_book.write(row)
                sock_failed = not opened

            if now >= next_can:
                can_now = parse_can(link_text(args.iface))
                timing = timing_key(can_now)
                if timing != prev_timing and any(timing):
                    row = context("timing")
                    row["detail"] = timing_detail(can_now)
                    can_book.write(row)
                    prev_timing = timing
                current = change_key(can_now)
                if current != prev_change and any(current):
                    row = context("state")
                    can_book.write(row)
                    prev_change = current
                    print(
                        f"{row['pi_time']}  {row['can_link']} {row['can_state']} "
                        f"busoff={row['can_bus_off']}",
                        flush=True,
                    )
                next_can = time.monotonic() + can_interval
                pull_can()

            if args.app_unit and now >= next_app:
                app = read_app(args.app_unit)
                if app is not None and app != prev_app:
                    row = context("app")
                    row.update(app)
                    mark_clear(row, CAN_APP_COLUMNS)
                    can_book.write(row)
                    prev_app = app
                next_app = time.monotonic() + 1.0

            if now >= next_status:
                fresh = read_cpu_jiffies()
                throttled = read_throttled()
                row = {
                    "pi_time": pi_now(),
                    "uptime_s": read_uptime(),
                    "cpu_temp_c": read_cpu_temp(),
                    "cpu_pct": cpu_percent(cpu, fresh),
                    "load1": read_load1(),
                    "mem_used_pct": read_mem_used_pct(),
                    "throttled": throttled,
                }
                row.update(power_now())
                row.update(can_now)
                row.update(age_fields(seen, time.monotonic()))
                status_book.write(row)
                cpu = fresh
                next_status = time.monotonic() + args.interval

            for text in kernel.read_lines():
                kern_time, message = split_kern(text)
                kern_book.write(
                    {"pi_time": pi_now(), "kern_time": kern_time, "message": message},
                    sync=False,
                )

            while link.fd is not None:
                try:
                    chunk = os.read(link.fd, 4096)
                except BlockingIOError:
                    break
                except OSError as exc:
                    link.close()
                    link.fail(uart_book, f"切断: {exc}", time.monotonic())
                    break
                if not chunk:
                    link.close()
                    link.fail(uart_book, "切断: 読み出しが終わりました", time.monotonic())
                    break
                link.on_bytes(chunk, uart_book)

            uart_book.sync_pending(now)
            can_book.sync_pending(now)
            kern_book.sync_pending(now)

            watch: list[int] = []
            if link.fd is not None:
                watch.append(link.fd)
            kern_fd = kernel.fileno()
            if kern_fd is not None:
                watch.append(kern_fd)
            if err_sock is not None:
                watch.append(err_sock.fileno())
            if id_sock is not None:
                watch.append(id_sock.fileno())
            soon = min(next_can, next_status, link.next_try if link.fd is None else next_can)
            timeout = max(0.0, min(0.2, soon - time.monotonic()))
            if watch:
                select.select(watch, [], [], timeout)
            else:
                time.sleep(timeout)
    except KeyboardInterrupt:
        return 0
    finally:
        link.close()
        kernel.close()
        for sock in (err_sock, id_sock):
            if sock is not None:
                sock.close()
        for book in (uart_book, status_book, can_book, kern_book):
            book.close()


if __name__ == "__main__":
    sys.exit(main())
