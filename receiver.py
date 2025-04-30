import serial
import struct
import csv
import datetime
import time

RED = "\033[31m"
RESET = "\033[0m"

PORT = "/dev/ttyUSB0"  # Change as needed
BAUD_RATE = 115200
TIMEOUT = 1
CSV_PATH = "log_standalone.csv"

def init_serial():
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=TIMEOUT)
        print("Serial port initialized")
        return ser
    except serial.SerialException as e:
        print(f"{RED}Failed to open serial port: {e}{RESET}")
        exit(1)

def read_serial_data(ser):
    try:
        size_bytes = ser.read(4)
        if len(size_bytes) != 4:
            print(f"{RED}Failed to read data size, flushing serial{RESET}")
            ser.reset_input_buffer()
            return None

        num_floats = struct.unpack("I", size_bytes)[0]
        print(f"Expecting {num_floats} floats")

        data_bytes = ser.read(num_floats * 4)
        if len(data_bytes) != num_floats * 4:
            print(f"{RED}Incomplete data received, flushing serial{RESET}")
            ser.reset_input_buffer()
            return None

        data = list(struct.unpack(f"{num_floats}f", data_bytes))
        print(f"Received {len(data)} floats: {data[0]}, {data[-1]}")
        print(f"Rssi: {data[1]} dBm, Snr: {data[2]}")
        return data
    except Exception as e:
        print(f"{RED}Serial read error: {e}{RESET}")
        return None

def write_to_csv(writer, data):
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    writer.writerow([timestamp] + data)
    print(f"Logged data at {timestamp}")

def main():
    ser = init_serial()
    with open(CSV_PATH, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "Data"])  # Add header once

        try:
            while True:
                data = read_serial_data(ser)
                if data is not None:
                    write_to_csv(writer, data)
                time.sleep(0.1)  # Prevents busy waiting
        except KeyboardInterrupt:
            print("Shutting down")
        finally:
            ser.close()

if __name__ == "__main__":
    main()
