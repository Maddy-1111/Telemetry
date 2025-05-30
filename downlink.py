import serial
import json
import os
import numpy as np


SERIAL_PORT = "/dev/ttyUSB1"
BAUD_RATE = 115200
TIMEOUT = 1
HEADER = b'\xDE\xAD\xBE\xEF'  ## same HEADER as sender ##


def generate_crc(byte_stream: bytes, poly=0x1021, init_val=0x0000):
    """Generate CRC-16-CCITT (XModem) [2 bytes] for the byte_stream"""
    crc = init_val
    for byte in byte_stream:
        crc ^= byte << 8
        for _ in range(8):
            if (crc & 0x8000):
                crc = (crc << 1) ^ poly
            else:
                crc <<= 1
            crc &= 0xFFFF  # Keep it 16-bit
    return crc.to_bytes(2, byteorder='little')


def flush_serial(ser):
    while ser.in_waiting > 0:
        ser.read(ser.in_waiting)


def read_packet(ser: serial.Serial):
    head_bytes = ser.read(len(HEADER))
    if head_bytes != HEADER:
        # print("header mismatch, dropping packet")
        flush_serial(ser)
        return None

    length_bytes = ser.read(2)
    if len(length_bytes) < 2:
        return None  # timeout or incomplete

    length = int.from_bytes(length_bytes, 'little')

    crc_bytes = ser.read(2)
    if len(crc_bytes) < 2:
        return None

    type_byte = ser.read(1)
    if len(type_byte) < 1:
        return None

    data_bytes = ser.read(length)
    if len(data_bytes) < length:
        return None

    calc_crc = generate_crc(type_byte + data_bytes)
    if calc_crc != crc_bytes:
        print("CRC check failed, dropping packet")
        flush_serial(ser)
        return None

    return {
        "type": type_byte.decode(),
        "data": data_bytes
    }


def unpack_flags(flag_bytes: bytes, total_bits: int, flags_list: list[str]) -> dict:
    bits = []
    for byte in flag_bytes:
        for bit_pos in range(8):
            bits.append(bool(byte & (1 << bit_pos)))
    bits = bits[:total_bits]  # truncate to exact flag count

    flag_values = {}
    for i, flag_key in enumerate(flags_list):
        flag_values[flag_key] = bits[i]
    return flag_values


def reverse_bytestream(data_bytes: bytes, output_order: list[str], fields: dict, flags: list[str], type_map: dict):
    data_buf = {}
    idx = 0

    for key in output_order:
        type_str = fields[key]["type"]

        if key == "Flags":
            total_bits = int(type_str.split("-")[1])
            num_bytes = (total_bits + 7) // 8
            flag_bytes = data_bytes[idx:idx+num_bytes]
            idx += num_bytes

            flag_values = unpack_flags(flag_bytes, total_bits, flags)
            data_buf.update(flag_values)

        else:
            dtype = type_map[type_str]
            size = dtype(0).nbytes  # numpy dtype size in bytes

            val_bytes = data_bytes[idx:idx+size]
            idx += size

            # Convert bytes back to numpy value
            value = np.frombuffer(val_bytes, dtype=dtype)[0]
            data_buf[key] = value

    return data_buf


def log_data(data_buf, filename="output_data1.jsonl"):
    # Convert numpy scalars to Python native types ####################### temp fix #####TODO: fix this
    clean_data = {k: (v.item() if hasattr(v, 'item') else v) for k, v in data_buf.items()}
    with open(filename, "a") as f:
        json_line = json.dumps(clean_data)
        f.write(json_line + "\n")


def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)

    current_dir = os.path.dirname(__file__)
    json_path = os.path.join(current_dir, 'packet_structure.json')

    with open(json_path, 'r') as f:
        structure = json.load(f)

    flags = structure["Flags"]
    fields = structure["Fields"]
    type_map = {
        "float16": np.float16,
        "float32": np.float32,
        "int16": np.int16,
        "int32": np.int32,
        "bool": bool
    }

    while True:
        packet = read_packet(ser)
        if packet is None:
            continue
        print(f"Received packet type: {packet['type']} data length: {len(packet['data'])}")
        
        if packet['type'] == 'A':
            output_order = structure['Output_Order_A']
        elif packet['type'] == 'B':
            output_order = structure['Output_Order_B']
        else:
            print(f"Type is {packet['type']}, doesn't match any known type")
            continue

        data_buf = reverse_bytestream(packet['data'], output_order, fields, flags, type_map)
        log_data(data_buf)
        print("logged data!!")


if __name__ == "__main__":
    main()
