#!/usr/bin/env python3

import serial
import time
import random

ser = serial.Serial('/dev/pts/0', 115200)

def send_movement(axis, value):
    """Send a movement command in the specified format: axis (8 bits), value (8 bits), end-of-package indicator (8 bits)."""
    msb = ((value & 0xFF00) >> 8)
    lsb = value & 0x0FF
    data = bytearray([axis, lsb, msb, 0xFF])  # 0xFF for -1 as end-of-package
    ser.write(data)

try:
    while True:
        # Randomly choose axis and value for demonstration
        axis = random.randint(0, 1)  # 0 for X, 1 for Y
        axis = 0
        value = random.randint(-127, 127)  # Random movement value
        value = int(input())
        send_movement(axis, value)
        time.sleep(1)  # Adjust sleep time as needed
except KeyboardInterrupt:
    print("Program terminated by user")
finally:
    ser.close()
