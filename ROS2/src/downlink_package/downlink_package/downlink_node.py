import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import struct

PORT = "/dev/ttyUSB0"  # Change as needed
BAUD_RATE = 115200
TIMEOUT = 1  # Adjust timeout as required
DATA_SIZE = 600
CHUNK_SIZE = 60


class SerialReceiverNode(Node):
    def __init__(self):
        super().__init__("serial_receiver")
        self.publisher_ = self.create_publisher(Float32MultiArray, "serial_data", 10)
        self.ser = serial.Serial(PORT, BAUD_RATE, timeout=TIMEOUT)
        self.get_logger().info("Serial receiver node initialized")
        self.final_data = []  # Store received data chunks

    def read_serial_data(self):
        while rclpy.ok():   
            try:
                size_bytes = self.ser.read(4)
                if len(size_bytes) != 4:
                    print("Failed to read data size, flushing serial")
                    self.ser.reset_input_buffer()  # Flush any remaining junk data
                    continue

                num_floats = struct.unpack("I", size_bytes)[0]  # Unsigned int
                print(f"Expecting {num_floats} floats")

                # Read the required number of bytes
                data_bytes = self.ser.read(num_floats * 4)
                if len(data_bytes) != num_floats * 4:
                    print("Incomplete data received, flushing serial")
                    self.ser.reset_input_buffer()  # Flush any remaining junk data
                    continue

                # Convert bytes to float array
                self.final_data = list(struct.unpack(f"{num_floats}f", data_bytes))
                print(f"Received {len(self.final_data)} floats: {self.final_data[0]}, {self.final_data[-1]}")
                print(f"Rssi: {self.final_data[1]} dBm, Snr: {self.final_data[2]}")


                # expectedIndex = 0
                # data_buffer = []
                # while expectedIndex < DATA_SIZE // CHUNK_SIZE:
                #     index_byte = self.ser.read(1)
                #     if not index_byte:
                #         self.ser.reset_input_buffer()
                #         break 
                #     index = struct.unpack('B', index_byte)[0]
                #     if index != expectedIndex:
                #         self.ser.reset_input_buffer()
                #         break

                #     data_bytes = self.ser.readline()
                #     num_floats = len(data_bytes) // 4
                #     print(num_floats)

                #     data_buffer.extend(list(struct.unpack(f'{num_floats}f', data_bytes)))

                #     print("recieved packet" + index)
                #     expectedIndex += 1

                # print("published data:" + str(self.final_data[0]) + "," + str(self.final_data[-1]))
                self.publish_data()


            except Exception as e:
                self.get_logger().error(f"Serial read error: {str(e)}")

    def publish_data(self):
        msg = Float32MultiArray()
        msg.data = self.final_data
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published {len(self.final_data)} floats")
        self.final_data.clear()


def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialReceiverNode()

    try:
        serial_node.read_serial_data()
    except KeyboardInterrupt:
        serial_node.get_logger().info("Shutting down serial receiver node")
    finally:
        serial_node.ser.close()
        serial_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
