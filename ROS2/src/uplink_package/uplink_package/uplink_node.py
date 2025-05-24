import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import serial
import struct
import json
import os
import numpy as np

GREEN = "\033[32m"
RESET = "\033[0m"

SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200
TIMEOUT = 1


class Telemetry_Node(Node):



    def __init__(self):
        super().__init__("telemetry_node")
        self.subscription = self.create_subscription(
            rosarray,
            "final_data",
            self.transmit_data,
            1)
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)


    type_map = {
        "float16": np.float16,
        "float32": np.float32,  # already float32 so float() is fine
        "int": int,
        # add more if needed
    }


    def load_structure(self):
        current_dir = os.path.dirname(__file__)
        json_path = os.path.join(current_dir, 'packet_structure.json')

        with open(json_path, 'r') as f:
            structure = json.load(f)

        self.input_order = structure["Input_Order"]
        self.output_order_A = structure["Output_Order_A"]
        self.output_order_B = structure["Output_Order_B"]
        self.fields = structure["Fields"]



    def transmit_data(self, msg):
        self.load_structure()

        self.data_inp = msg.data
        self.data_buf = {}

        # print(len(self.data_inp))
        # print(len(self.input_order))

        for i in range(len(self.data_inp)):
            key = self.input_order[i]
            value = self.data_inp[i]
            type_str = self.fields[key]["type"]

            if type_str.startswith("custom-"):
                num_bits = int(type_str.split("-")[1])
                bit_flags = [(int(value) >> bit) & 1 for bit in range(num_bits)]

                for j in range(num_bits):
                    flag_key = f"{key.replace('_Flags', '_Flag')}{j+1}"
                    self.data_buf[flag_key] = bool(bit_flags[j])
            else:
                self.data_buf[key] = self.type_map[type_str](value)


        # for key in self.input_order[:11]:
        #     print(f"{key}: {self.data_buf[key]} , type = {type(self.data_buf[key])}")
        for key in self.data_buf:
            print(f"{key}: {self.data_buf[key]} , type = {type(self.data_buf[key])}")
        print("")

        # self.data = msg.data
        # total_floats = len(self.data)
        
        # length_byte = total_floats.to_bytes(1, 'little')
        # data_bytes = self.data.tobytes()  # Convert float32 array to bytes
        
        # self.ser.write(length_byte + data_bytes)  # Send bytes over serial
        # print(f"Sent data : {len(data_bytes)} bytes")

    #     response = self.ser.readline()
    #     if response == b'':
    #         print("no ack :(")
    #     elif response == b'ack\r\n':  # Compare bytes properly
    #         print("------")
    #     else:
    #         print("xxxxxx")
    #         print(response)

    # ########## TODO: remove this after removing all other printf statements (adding a delay of 1s)
    #     response = self.ser.readline()

    #     while(response != b''):
    #         decoded_response = response.decode().strip()
    #         print(f"{GREEN}Decoded response: {decoded_response}{RESET}")
    #         response = self.ser.readline()
            
    # ###########



    
    #### TODO: this is returning a 16 bit? ckeck it ####
    def generate_crc(self, data_floats):
        """Generate CRC-16 (Modbus) for a list of float32 values."""
        crc = 0xFFFF

        for value in data_floats:
            byte_array = struct.pack('<f', value)  # Convert float to little-endian bytes
            for byte in byte_array:
                crc ^= byte
                for _ in range(8):
                    if crc & 0x01:
                        crc = (crc >> 1) ^ 0xA001
                    else:
                        crc >>= 1

        return crc



def main(args=None):
    rclpy.init(args=args)

    telemetry_node = Telemetry_Node()

    rclpy.spin(telemetry_node)

    telemetry_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
