import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import serial

CYAN = "\033[36m"
YELLOW = "\033[33m"
GREEN = "\033[32m"
RESET = "\033[0m"

class SERIAL_NODE(Node):

    def __init__(self,node):
        super().    __init__(node)
    
    # final Data Subscriber
    def init_final_data_subscriber(self, topic, serial_port,baud_rate):
        self.final_data_subscriber = self.create_subscription(
            rosarray, topic, self.receive_final_data, 10
        )
        self.final_data_subscriber  # prevent unused variable warning
        self.final_sub_data = None
        self.ser = serial.Serial(serial_port, baud_rate, timeout=3)


    def receive_final_data(self, msg):  
        self.final_sub_data = msg.data
        data = ','.join(map(str, self.final_sub_data))
        self.ser.write(data.encode())
        print(f'Sent: {self.final_sub_data[:3]} ... {self.final_sub_data[-2:]}')
        

        ####### Doesn't work, idk why ######
        # while self.ser.in_waiting:  # Check if data is available
        #     response = self.ser.readline()  # Read the response as raw bytes

        #     try:
        #         decoded_response = response.decode().strip()  # Attempt to decode
        #         print(f"{GREEN}Decoded response: {decoded_response}{RESET}")
        #     except UnicodeDecodeError:
        #         print("Received non-UTF-8 data.") 
        #######################################


        # Read the response as raw bytes
        # temp = self.ser.in_waiting
        response = self.ser.readline()

        while(response != b''):
            try:
                # Attempt to decode if it's expected to be a valid UTF-8 string
                decoded_response = response.decode().strip()
                print(f"{GREEN}Decoded response: {decoded_response}{RESET}")
            except UnicodeDecodeError:
                # Handle or log the error if the data is not valid UTF-8
                print("Received non-UTF-8 data.")
            # response = 0
            response = self.ser.readline()
            # print(response)


def main(args=None):
    rclpy.init(args=args)

    serial_node = SERIAL_NODE("serial_node")
    serial_node.init_final_data_subscriber("final_data","/dev/ttyUSB0",115200)

    while rclpy.ok():
        rclpy.spin_once(serial_node) 
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    serial_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
