import serial
import pyautogui

ser = serial.Serial('COM8', 115200)

try:
    # sync package
    while True:
        print('Waiting for sync package...')
        while True:
            data = ser.read(1)
            if data == b'\xff':
                break

        Read 4 bytes from UART
        data = ser.read(1)
        print(data)





        
        if data == b'\x01':
            print('mouse click')
            pyautogui.mouseDown()
        else:
            pyautogui.mouseUp()

except KeyboardInterrupt:
    print("Program terminated by user")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    ser.close()