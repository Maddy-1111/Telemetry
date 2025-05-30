import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import serial
import json
import os
import numpy as np


SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200
TIMEOUT = 1
HEADER = b'\xDE\xAD\xBE\xEF'        ## DEADBEEF ##


class Telemetry_Node(Node):

    type_map = {
        "float16": np.float16,
        "float32": np.float32,
        "int16": np.int16,
        "int32": np.int32,
        "bool": bool
    }

    def __init__(self):
        super().__init__("telemetry_node")
        self.subscription = self.create_subscription(
            rosarray,
            "final_data",
            self.transmit_data,
            1)
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        self.ser.reset_input_buffer()
        self.msg_count = 0
        self.load_structure()


    def load_structure(self):
        current_dir = os.path.dirname(__file__)
        json_path = os.path.join(current_dir, 'packet_structure.json')

        with open(json_path, 'r') as f:
            structure = json.load(f)

        self.input_order = structure["Input_Order"]
        self.output_order_A = structure["Output_Order_A"]
        self.output_order_B = structure["Output_Order_B"]
        self.flags = structure["Flags"]
        self.fields = structure["Fields"]

        self.data_buf = {}

    ### To initialise the data_buf with all 0s ###
        # for key in self.input_order:
        #     type_str = self.fields[key]["type"]

        #     if type_str.startswith("custom-"):
        #         num_bits = int(type_str.split("-")[1])
        #         for j in range(num_bits):
        #             flag_key = f"{key.replace('_Flags', '_Flag')}{j+1}"
        #             self.data_buf[flag_key] = False  # initialize custom flags to False
        #     else:
        #         self.data_buf   # 0 with correct dtype


    def transmit_data(self, msg):
        self.data_in = msg.data

        """Create the buffer dictionary"""
        for i in range(len(self.data_in)):
            key = self.input_order[i]
            value = self.data_in[i]
            type_str = self.fields[key]["type"]

            if type_str.startswith("custom-"):
                num_bits = int(type_str.split("-")[1])
                bit_flags = [(int(value) >> bit) & 1 for bit in range(num_bits)]        ####### TODO: check if its (int) or (float)

                for j in range(num_bits):
                    flag_key = f"{key.replace('_Flags', '_Flag')}{j+1}"
                    self.data_buf[flag_key] = bool(bit_flags[j])
            else:
                self.data_buf[key] = self.type_map[type_str](value)


        # for key in self.data_buf:
        #     print(f"{key}: {self.data_buf[key]} , type = {type(self.data_buf[key])}")
        # print("")

        """Generate, format and send the required bytestream"""
        self.msg_count = (self.msg_count + 1) % 100_000
        if self.msg_count % 10 == 0:
            self.data_out = self.generate_bytestream(self.output_order_B)
            self.type = 'B'.encode()
        else:
            self.data_out = self.generate_bytestream(self.output_order_A)
            self.type = 'A'.encode()

        self.crc = self.generate_crc(self.type + self.data_out)
        self.length = len(self.data_out).to_bytes(2, 'little')
        self.packet = HEADER + self.length + self.crc + self.type + self.data_out
        self.ser.write(self.packet)
        print(f"Packet sent: length = {len(self.packet)}, 'length'={len(self.data_out)}")

        ########## TODO: remove this after removing all other printf statements (adding a delay of 1s)

        response = self.ser.readline()

        while response != b'':
            try:
                decoded_response = response.decode().strip()
                print(f"Decoded response: {decoded_response}")
            except UnicodeDecodeError:
                print("Warning: Received non-UTF-8 data, skipping line.")
            response = self.ser.readline()
                
        ###########

    def generate_bytestream(self, output_order):
        byte_stream = bytearray()

        for key in output_order:
            type_str = self.fields[key]["type"]

            if key == "Flags":
                total_bits = int(type_str.split("-")[1])
                flags = [self.data_buf[flag_key] for flag_key in self.flags]

                byte_stream.extend(self.pack_flags(flags, total_bits))

            else:
                value = self.data_buf[key]
                byte_stream.extend(self.type_map[type_str](value).tobytes())

        return bytes(byte_stream)
    

    def pack_flags(self, flags: list[bool], total_bits: int) -> bytes:
        flags += [False] * (total_bits - len(flags))
        
        packed = bytearray()
        for i in range(0, total_bits, 8):
            byte = 0
            for bit in range(8):
                if flags[i + bit]:
                    byte |= (1 << bit)
            packed.append(byte)
        
        return bytes(packed)


    def generate_crc(self, byte_stream: bytes, poly=0x1021, init_val=0x0000):
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



def main(args=None):
    rclpy.init(args=args)

    telemetry_node = Telemetry_Node()

    rclpy.spin(telemetry_node)

    telemetry_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
