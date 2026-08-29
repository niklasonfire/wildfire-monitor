#!/usr/bin/env python3
"""Drive the wildfire_monitor console over the serial link.

The firmware answers every command with tagged single-line records and then
reprints its prompt, so this script can send a command, collect everything up
to the next prompt, and hand back a deterministic block of output. That makes
a capture session scriptable instead of something to be watched by hand.

    ./wf.sh info ping
    ./wf.sh --reset 'scan 10' devs 'dev 0'
    ./wf.sh 'connect name DL' 'probe 30@60' 'rec dump'

A command may carry a per-command timeout with '@seconds'. Everything read is
also appended to the capture file when --capture is given.
"""
import argparse
import os
import sys
import time

import serial

# In its dumb-terminal mode the console prints "wf> " plus a trailing
# space, so the prompt is matched against the right-stripped buffer.
PROMPT = b"wf>"


def hw_reset(ser):
    """Pulse EN via RTS. DTR must stay high or the ESP32 enters download mode."""
    ser.dtr = False
    ser.rts = True
    time.sleep(0.1)
    ser.rts = False


def read_until_prompt(ser, timeout, out):
    """Reads until the prompt reappears at the end of the buffer, or timeout."""
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
            if out is not None:
                out.write(chunk)
                out.flush()
            # The prompt is the last thing printed once a command has finished.
            if buf.rstrip().endswith(PROMPT):
                return buf.decode(errors="replace"), True
        else:
            time.sleep(0.01)
    return buf.decode(errors="replace"), False


def split_timeout(cmd, default):
    if "@" in cmd:
        head, _, tail = cmd.rpartition("@")
        try:
            return head, float(tail)
        except ValueError:
            pass
    return cmd, default


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("commands", nargs="*", help="console commands, 'cmd@secs' to set a timeout")
    ap.add_argument("--port", default=os.environ.get("ESPPORT", "/dev/ttyACM0"))
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=30.0, help="default per-command timeout")
    ap.add_argument("--reset", action="store_true", help="reset the board first")
    ap.add_argument("--boot-wait", type=float, default=3.0, help="seconds to drain after a reset")
    ap.add_argument("--listen", type=float, default=0.0, help="stream output for N seconds and exit")
    ap.add_argument("--capture", help="append the raw session to this file")
    args = ap.parse_args()

    out = open(args.capture, "ab") if args.capture else None
    ser = serial.Serial(args.port, args.baud, timeout=0.1)

    try:
        if args.reset:
            hw_reset(ser)
            text, _ = read_until_prompt(ser, args.boot_wait, out)
            sys.stdout.write(text)

        if args.listen > 0:
            deadline = time.time() + args.listen
            while time.time() < deadline:
                chunk = ser.read(ser.in_waiting or 1)
                if chunk:
                    sys.stdout.write(chunk.decode(errors="replace"))
                    sys.stdout.flush()
                    if out is not None:
                        out.write(chunk)
            return 0

        # A newline first: it flushes any half-typed line and gets a prompt
        # printed, which is the synchronisation point for everything after.
        ser.write(b"\n")
        read_until_prompt(ser, 2.0, out)

        rc = 0
        for raw in args.commands:
            cmd, timeout = split_timeout(raw, args.timeout)
            sys.stdout.write(f"\n=== $ {cmd}\n")
            sys.stdout.flush()
            if out is not None:
                out.write(f"\n=== $ {cmd}\n".encode())
            ser.reset_input_buffer()
            # A bare \n: sending \r\n makes linenoise read a second, empty line.
            ser.write((cmd + "\n").encode())
            ser.flush()
            text, ok = read_until_prompt(ser, timeout, out)
            sys.stdout.write(text)
            if not ok:
                sys.stdout.write(f"\n!!! timeout after {timeout}s\n")
                rc = 1
            sys.stdout.flush()
        sys.stdout.write("\n")
        return rc
    finally:
        ser.close()
        if out is not None:
            out.close()


if __name__ == "__main__":
    sys.exit(main())
