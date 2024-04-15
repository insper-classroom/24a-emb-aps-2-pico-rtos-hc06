import serial
from pynput.keyboard import Key, Controller
import time

ser = serial.Serial('/dev/ttyACM0', 115200)
keyboard = Controller()

def parse_data(data):
    axis = data[0]
    value = int.from_bytes(data[1:3], byteorder='little', signed=True)
    return axis, value

def press_key(axis, value):
    if value == 0 and axis != 2:
        return  # Não faz nada se o valor for 0 e não for o terceiro eixo

    if axis == 0:  # X-axis
        key = 'd' if value < 0 else 'a'
    elif axis == 1:  # Y-axis
        key = 'w' if value > 0 else 's'
    elif axis == 2:  # Third axis for additional keys
        if value == 0:
            key = 'e'  # 'E' key action
        elif value == 1:
            key = 'q'  # 'Q' key action

    # Press and release key
    keyboard.press(key)
    time.sleep(0.1)  # Hold key for 100 ms
    keyboard.release(key)

try:
    print('Waiting for sync package...')
    while True:
        data = ser.read(1)
        if data == b'\xff':
            data = ser.read(3)
            axis, value = parse_data(data)
            press_key(axis, value)

except KeyboardInterrupt:
    print("Program terminated by user")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    ser.close()