import serial
import time

SERIAL_PORT = "/dev/ttyUSB1"
BAUD_RATE = 115200
TIMEOUT = 1  # in seconds

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)

while True:
    if ser.in_waiting:  # if data is available
        data = ser.read(ser.in_waiting)  # read all available bytes
        print("Got:", data.decode('utf-8', errors='replace'))  # decode and print

        time.sleep(0.5)
