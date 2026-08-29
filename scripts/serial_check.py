#!/usr/bin/env python3
"""Non-interactive smoke test for the bring-up firmware.

Resets the board, waits for the console banner, then sends a few commands and
prints the replies. Run inside the IDF container:

    ./idf.sh shell -c "python scripts/serial_check.py"
"""
import os
import sys
import time

import serial

PORT = os.environ.get("ESPPORT", "/dev/ttyACM0")
COMMANDS = ["info", "ping", "led on", "btn", "led off", "ping"]


def drain(ser, seconds):
    end = time.time() + seconds
    out = b""
    while time.time() < end:
        out += ser.read(ser.in_waiting or 1)
    return out.decode(errors="replace")


def main():
    ser = serial.Serial(PORT, 115200, timeout=0.2)

    # Hardware reset: EN is driven by RTS, DTR must stay high so the board does
    # not enter the download mode.
    ser.dtr = False
    ser.rts = True
    time.sleep(0.1)
    ser.rts = False

    print(f"--- boot output on {PORT} ---")
    print(drain(ser, 3.0), end="")

    for cmd in COMMANDS:
        ser.reset_input_buffer()
        ser.write((cmd + "\r\n").encode())
        ser.flush()
        reply = drain(ser, 0.6)
        print(f"--- $ {cmd}")
        print(reply, end="" if reply.endswith("\n") else "\n")

    ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
