#!/usr/bin/env python3
# File Name: rover_fault.py
# Version: 0.1.1
# Description: Rover MPPT の故障ビットマップ（保持レジスタ 0x0121 / 0x0122）を読む。
# Development Environment:
#   Raspberry Pi（/dev/serial0, GPIO27 = DE/RE）
#   Python 3
# Dependencies:
#   pyserial
#   RPi.GPIO
#
# 電圧読み取り（0x0101 を 1 ワード）と同じバス設定。
# 要求は 8 バイト。write() の戻りはシフト完了ではないので、
# tcdrain のあと 4 ms 待ってから DE を受信に戻す。
# 正常応答は 9 バイト（データ 4 バイト）。例外応答は 5 バイト。
# 無応答のときは 0x0101 と 0x0120〜0x0122 を 1 ワードずつ読んで切り分ける。
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
# 最終ビットが出てから受信に戻すまでの待ち。3.5 文字時間（約 3.6 ms）より少し長い。
TX_TURNAROUND = 0.004

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


def build_request(addr: int = FAULT_ADDR, count: int = READ_REGISTER_COUNT) -> bytes:
	frame = bytes([
		SLAVE_ID,
		0x03,
		(addr >> 8) & 0xFF,
		addr & 0xFF,
		(count >> 8) & 0xFF,
		count & 0xFF,
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
	# write() はカーネルへ渡した時点で戻る。シフトが終わるまで DE を維持する。
	ser.flush()
	time.sleep(TX_TURNAROUND)

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


def parse_registers(response: bytes, count: int) -> list[int] | None:
	# 例外は 5 バイト。正常は ID + FC + バイト数 + データ + CRC 2。
	if len(response) < 5:
		print(f"NG: response length error: {len(response)} bytes")
		return None

	if not check_header(response):
		return None

	if response[2] != count * 2 or len(response) != 5 + count * 2:
		print(f"NG: unexpected byte count: {response[2]}")
		return None

	words = []
	for i in range(count):
		off = 3 + (i * 2)
		words.append((response[off] << 8) | response[off + 1])
	return words


def parse_fault_bitmap(response: bytes) -> int | None:
	words = parse_registers(response, READ_REGISTER_COUNT)
	if words is None:
		return None
	return (words[0] << 16) | words[1]


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


def read_registers(ser: serial.Serial, addr: int, count: int) -> list[int] | None:
	response = send_request_and_read(ser, build_request(addr, count))
	if response is None:
		return None
	return parse_registers(response, count)


def diagnose_timeout(ser: serial.Serial):
	"""0x0121 が無応答のとき、バスと隣接レジスタのどちらが黙っているかを分ける。"""
	print("電圧（0x0101）でバスを確認します")
	voltage = read_registers(ser, 0x0101, 1)
	if voltage is None:
		print("電圧も無応答です。Rover がこの UART に応答していません")
		return

	print(f"Battery Voltage: {voltage[0] * 0.1:.1f} V")
	print("0x0120 から 0x0122 を 1 ワードずつ読みます")
	got = False
	for addr in (0x0120, 0x0121, 0x0122):
		words = read_registers(ser, addr, 1)
		if words is None:
			continue
		got = True
		print(f"0x{addr:04X}: 0x{words[0]:04X}")

	if not got:
		print("電圧は返ります。0x0120〜0x0122 はこの個体から返りません")


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
				words = read_registers(ser, FAULT_ADDR, READ_REGISTER_COUNT)

				if words is not None:
					print_fault_bitmap((words[0] << 16) | words[1])
				else:
					print("Fault bitmap: read failed")
					diagnose_timeout(ser)

			except Exception as e:
				print("Exception:", e)

	finally:
		GPIO.output(DE_RE_PIN, GPIO.LOW)
		ser.close()
		GPIO.cleanup()


if __name__ == "__main__":
	main()
