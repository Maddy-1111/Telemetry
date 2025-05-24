import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import random

class ExamplePublisher(Node):
    def __init__(self, array_length):
        super().__init__('example_publisher')
        self.publisher = self.create_publisher(rosarray, 'final_data', 10)
        self.array_length = array_length
        self.counter = 0
        self.timer = self.create_timer(1.0, self.publish_data)  # Publish every second

    def publish_data(self):
        # Create an array with the specified length
        data = [random.uniform(0, 100) for _ in range(self.array_length)]
        
        # Set the first and last elements as counters
        data[0] = float(self.counter)
        data[10] = float(self.counter) # This is for the output packet
        data[-1] = float(self.counter)
        
        # Increment the counter for the next array
        self.counter += 1
        
        # Create the Float32MultiArray message
        msg = rosarray()
        msg.data = data
        
        # Publish the message
        self.publisher.publish(msg)
        
        # Log the publishing action (only showing the first and last 5 values for readability)
        self.get_logger().info(f'Publishing ({self.array_length} floats): {msg.data[:3]} ... {msg.data[-2:]}')

def main(args=None):
    rclpy.init(args=args)
    example_publisher = ExamplePublisher(array_length=111)
    
    try:
        rclpy.spin(example_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        example_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()