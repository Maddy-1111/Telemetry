import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import serial
import struct

CYAN = "\033[36m"
YELLOW = "\033[33m"
GREEN = "\033[32m"
RESET = "\033[0m"

CHUNK_SIZE = 60

class SERIAL_NODE(Node):

    def __init__(self,node):
        super().    __init__(node)
    
    def init_final_data_subscriber(self, topic, serial_port,baud_rate):
        self.final_data_subscriber = self.create_subscription(
            rosarray, topic, self.send_final_data, 10
        )
        self.final_data_subscriber
        self.final_sub_data = None
        self.ser = serial.Serial(serial_port, baud_rate, timeout=2)


    def send_final_data(self, msg): 
        self.final_sub_data = msg.data  # msg.data is of type `array.array`
        total_floats = len(self.final_sub_data)
        
        for i in range(0, total_floats, CHUNK_SIZE):
            chunk = self.final_sub_data[i:i + CHUNK_SIZE]
            # byte_data = b''.join(struct.pack('<f', value) for value in chunk)  # Little-endian
            length_byte = len(chunk).to_bytes(1, 'little')
            byte_data = chunk.tobytes()  # Convert float32 array to bytes
            
            self.ser.write(length_byte + byte_data)  # Send bytes over serial

            print(f"Sent chunk {i // CHUNK_SIZE + 1}: {len(byte_data)} bytes")

            response = self.ser.readline()
            if response == b'':
                print("no ack :(")
            elif response == b'ack\r\n':  # Compare bytes properly
                print("------")
                continue
            else:
                print("How tf did you get here?")
                print(response)


        response = self.ser.readline()

        while(response != b''):
            decoded_response = response.decode().strip()
            print(f"{GREEN}Decoded response: {decoded_response}{RESET}")
            # response = 0
            response = self.ser.readline()
            # print(response)

    
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

    serial_node = SERIAL_NODE("serial_node")
    serial_node.init_final_data_subscriber("final_data","/dev/ttyUSB0",115200)

    while rclpy.ok():
        rclpy.spin_once(serial_node) 

    serial_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
