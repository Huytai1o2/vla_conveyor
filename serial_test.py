import time
import serial

PORT = "COM18"
BAUD = 115200

board = serial.Serial(PORT, BAUD, timeout=0.25, write_timeout=1)
time.sleep(2.5)
board.reset_input_buffer()

board.write(b"PING\n")
board.flush()

deadline = time.monotonic() + 2
while time.monotonic() < deadline:
    line = board.readline().decode(errors="ignore").strip()
    if line:
        print("YoloUNO ->", line)
    if line == "PONG":
        break
else:
    board.close()
    raise RuntimeError("Không nhận được PONG.")

board.reset_input_buffer()
board.write(b"MOVE,RIGHT,300\n")
board.flush()
print("TX -> MOVE,RIGHT,300")

deadline = time.monotonic() + 3
while time.monotonic() < deadline:
    line = board.readline().decode(errors="ignore").strip()
    if line:
        print("YoloUNO ->", line)
    if line == "DONE":
        break
else:
    board.write(b"STOP\n")
    raise RuntimeError("Không nhận được DONE.")

board.close()
print("Serial và motor test hoàn tất.")
