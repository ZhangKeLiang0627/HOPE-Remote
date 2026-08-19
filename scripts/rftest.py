#!/usr/bin/env python3
"""HOPE-Remote 串口自动化自测：COM4 @ 115200"""
import sys, time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"
BAUD = 115200

def send(ser, cmd, wait=1.0):
    ser.reset_input_buffer()
    ser.write((cmd + "\r").encode())
    time.sleep(wait)
    out = b""
    end = time.time() + wait
    while time.time() < end:
        n = ser.in_waiting
        if n:
            out += ser.read(n)
        time.sleep(0.05)
    return out.decode(errors="replace").strip()

def main():
    ser = serial.Serial(PORT, BAUD, timeout=2)
    time.sleep(0.5)
    print("=== boot ===")
    print(send(ser, "", 1.2) or "(no output yet)")
    print("=== help ===")
    print(send(ser, "help", 1.5))
    print("=== evtest ===")
    print(send(ser, "evtest", 1.5))
    print("=== slots (before) ===")
    print(send(ser, "slots", 1.5))
    print("=== sc100 55C311 ===")
    print(send(ser, "sc100 55C311", 1.5))
    print("=== du100 ===")
    print(send(ser, "du100", 1.2))
    print("=== rfloop ===")
    print(send(ser, "rfloop", 6.0))
    print("=== fs100 ===")
    print(send(ser, "fs100", 3.0))
    print("=== slots (after) ===")
    print(send(ser, "slots", 1.5))
    ser.close()

if __name__ == "__main__":
    main()
