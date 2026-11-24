#!/usr/bin/env python3
# File Name: rover_fault.py
# Version: 0.1.0
# Description: Rover MPPT の故障ビットマップ（保持レジスタ 0x0121 / 0x0122）を読む。
# Development Environment:
#   Raspberry Pi（/dev/serial0, GPIO27 = DE/RE）
#   Python 3
# Dependencies:
#   pyserial
#   RPi.GPIO
#
# 電圧読み取り（0x0101 を 1 ワード）と同じバス設定。
# 要求は 8 バイトのままなので、送信完了待ちは電圧スクリプトと同じ 9 ms。
# 正常応答は 9 バイト（データ 4 バイト）。例外応答は 5 バイト。
#
# ビットの意味は世代で 16 ビットずれる。
#   Renogy 2025: 主なフラグは下位（0x0122）
#   Classic Rover 20A/40A: 主なフラグは上位（0x0121）、下位は予約
# 両方を出す。画面のエラーと突き合わせて、どちらの列を使うか決める。
# Classic の資料は B17 と B18 の説明が表と例で逆。ここはアドレス表に合わせる。

import serial
import time
import RPi.GPIO as GPIO

SERIAL_PORT = "/dev/serial0"
BAUDRATE = 9600

DE_RE_PIN = 27
SLAVE_ID = 0xFF

FAULT_ADDR = 0x0121
READ_REGISTER_COUNT = 2

FRAME_SILENT_TIME = 0.05
RESPONSE_TIMEOUT = 1.0
# 9600 8N1 で 8 バイト送る時間（10 bit/byte）に 1 ms 足した値。
TX_WAIT_TIME = 0.009

# 立っているビットだけを、この順で表示する。(Renogy 2025, Classic Rover)
FAULT_BITS = (
	("バッテリー過放電", "予約"),
	("バッテリー過電圧", "予約"),
	("低電圧警告", "予約"),
	("負荷短絡", "予約"),
	("負荷過電力または過電流", "予約"),
	("コントローラ温度过高", "予約"),
	("周囲温度过高", "予約"),
	("PV入力過電力", "予約"),
	("PV入力短絡", "予約"),
	("PV入力過電圧", "予約"),
	("パネル逆電流", "予約"),
	("パネル動作点過電圧", "予約"),
	("パネル逆接続", "予約"),
	("バッテリー逆接続", "予約"),
	("充電MOS短絡", "予約"),
	("ファン異常", "予約"),
	("バッテリー低温保護", "バッテリー過放電"),
	("バッテリー短絡保護", "バッテリー過電圧"),
	("予約", "低電圧警告"),
	("予約", "負荷短絡"),
	("予約", "負荷過電力または過電流"),
	("予約", "コントローラ温度过高"),
	("予約", "周囲温度过高"),
	("予約", "PV入力過電力"),
	("予約", "PV入力短絡"),
	("予約", "PV入力過電圧"),
	("予約", "パネル逆電流"),
	("予約", "パネル動作点過電圧"),
	("予約", "パネル逆接続"),
	("予約", "逆流防止MOS短絡"),
	("予約", "充電MOS短絡"),
	("予約", "予約"),
)

EXCEPTION_TEXT = {
	0x01: "未対応の機能コード",
	0x02: "不正なアドレス",
	0x03: "データ長が範囲外",
	0x04: "読み書き失敗",
	0x05: "チェック不一致",
}


def modbus_crc16(data: bytes) -> int:
	crc = 0xFFFF

	for b in data:
		crc ^= b
		for _ in range(8):
			if crc & 0x0001:
				crc >>= 1
				crc ^= 0xA001
			else:
				crc >>= 1

	return crc


def add_crc(frame: bytes) -> bytes:
	crc = modbus_crc16(frame)
	return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def print_hex(label: str, data: bytes):
	print(f"{label}: {data.hex(' ')}")


def build_request() -> bytes:
	frame = bytes([
		SLAVE_ID,
		0x03,
		(FAULT_ADDR >> 8) & 0xFF,
		FAULT_ADDR & 0xFF,
		(READ_REGISTER_COUNT >> 8) & 0xFF,
		READ_REGISTER_COUNT & 0xFF,
	])
	return add_crc(frame)


def expected_length(data: bytes) -> int | None:
	"""先頭 3 バイトが揃ってから、フレーム長を決める。"""
	if len(data) < 3:
		return None
	if data[1] & 0x80:
		return 5
	if data[1] == 0x03:
		return 5 + data[2]
	return None


def read_response(ser: serial.Serial, timeout: float) -> bytes:
	start = time.time()
	data = bytearray()

	while time.time() - start < timeout:
		n = ser.in_waiting

		if n > 0:
			data.extend(ser.read(n))
			start = time.time()
			need = expected_length(data)
			if need is not None and len(data) >= need:
				break

		time.sleep(0.001)

	return bytes(data)


def send_request_and_read(ser: serial.Serial, request: bytes) -> bytes | None:
	GPIO.output(DE_RE_PIN, GPIO.LOW)
	time.sleep(FRAME_SILENT_TIME)

	ser.reset_input_buffer()
	ser.reset_output_buffer()

	print_hex("TX", request)

	GPIO.output(DE_RE_PIN, GPIO.HIGH)
	time.sleep(0.0001)

	ser.write(request)
	time.sleep(TX_WAIT_TIME)

	GPIO.output(DE_RE_PIN, GPIO.LOW)

	response = read_response(ser, RESPONSE_TIMEOUT)

	if not response:
		print("RX: timeout")
		return None

	print_hex("RX", response)
	return response


def check_header(response: bytes) -> bool:
	received_crc = response[-2] | (response[-1] << 8)
	calculated_crc = modbus_crc16(response[:-2])

	if received_crc != calculated_crc:
		print("NG: CRC error")
		print(f"Received CRC: 0x{received_crc:04X}")
		print(f"Calculated CRC: 0x{calculated_crc:04X}")
		return False

	if response[0] != SLAVE_ID:
		print(f"NG: unexpected slave id: 0x{response[0]:02X}")
		return False

	if response[1] & 0x80:
		code = response[2]
		text = EXCEPTION_TEXT.get(code, "不明")
		print(f"Exception response: code=0x{code:02X} ({text})")
		return False

	if response[1] != 0x03:
		print(f"NG: unexpected function code: 0x{response[1]:02X}")
		return False

	return True


def parse_fault_bitmap(response: bytes) -> int | None:
	# 例外は 5 バイト。正常は ID + FC + バイト数 + データ 4 + CRC 2。
	if len(response) < 5:
		print(f"NG: response length error: {len(response)} bytes")
		return None

	if not check_header(response):
		return None

	if response[2] != 4:
		print(f"NG: unexpected byte count: {response[2]}")
		return None

	if len(response) != 9:
		print(f"NG: response length error: {len(response)} bytes")
		return None

	high = (response[3] << 8) | response[4]
	low = (response[5] << 8) | response[6]
	return (high << 16) | low


def print_fault_bitmap(bitmap: int):
	high = (bitmap >> 16) & 0xFFFF
	low = bitmap & 0xFFFF
	print(f"0x0121: 0x{high:04X}")
	print(f"0x0122: 0x{low:04X}")
	print(f"bitmap: 0x{bitmap:08X}")

	if bitmap == 0:
		print("故障ビットは立っていません")
		return

	print("bit  Renogy 2025（下位が本体）        Classic Rover（上位が本体）")
	for bit in range(32):
		if bitmap & (1 << bit):
			renogy, classic = FAULT_BITS[bit]
			print(f"B{bit:<3} {renogy:<28} {classic}")


def read_fault_bitmap(ser: serial.Serial) -> int | None:
	request = build_request()
	response = send_request_and_read(ser, request)

	if response is None:
		return None

	return parse_fault_bitmap(response)


def main():
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(DE_RE_PIN, GPIO.OUT, initial=GPIO.LOW)

	ser = serial.Serial(
		port=SERIAL_PORT,
		baudrate=BAUDRATE,
		bytesize=serial.EIGHTBITS,
		parity=serial.PARITY_NONE,
		stopbits=serial.STOPBITS_ONE,
		timeout=0.02,
	)

	print("Rover fault bitmap reader")
	print(f"ID=0x{SLAVE_ID:02X}  addr=0x{FAULT_ADDR:04X}  count={READ_REGISTER_COUNT}")

	try:
		while True:
			input("\nEnterキーで取得 > ")

			try:
				bitmap = read_fault_bitmap(ser)

				if bitmap is not None:
					print_fault_bitmap(bitmap)
				else:
					print("Fault bitmap: read failed")

			except Exception as e:
				print("Exception:", e)

	finally:
		GPIO.output(DE_RE_PIN, GPIO.LOW)
		ser.close()
		GPIO.cleanup()


if __name__ == "__main__":
	main()
