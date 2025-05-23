import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import serial
import struct

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
            10)
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)


    def transmit_data(self, msg): 
        self.data = msg.data[8:]
        total_floats = len(self.data)
        
        # data_bytes = b''.join(struct.pack('<f', value) for value in self.data)  # Little-endian
        length_byte = len(self.data).to_bytes(1, 'little')
        data_bytes = self.data.tobytes()  # Convert float32 array to bytes
        
        self.ser.write(length_byte + data_bytes)  # Send bytes over serial

        print(f"Sent data : {len(data_bytes)} bytes")

        response = self.ser.readline()
        if response == b'':
            print("no ack :(")
        elif response == b'ack\r\n':  # Compare bytes properly
            print("------")
        else:
            print("xxxxxx")
            print(response)

    ########## TODO: remove this after removing all other printf statements (adding a delay of 1s)
        response = self.ser.readline()

        while(response != b''):
            decoded_response = response.decode().strip()
            print(f"{GREEN}Decoded response: {decoded_response}{RESET}")
            response = self.ser.readline()
            
    ###########


    
    #### TODO: this is returning a 16 bit? ckeck it ### 
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
