import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import csv
import datetime

class DataLoggerNode(Node):
    def __init__(self):
        super().__init__("data_logger_node")
        self.subscription = self.create_subscription(
            Float32MultiArray,
            "serial_data",
            self.data_callback,
            10
        )
        self.subscription  # Prevent unused variable warning
        self.file_path = "/home/madhav/Desktop/Agnirath/Telemetry/log.csv"
        self.file = open(self.file_path, "a", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["Timestamp", "Data"])
        self.get_logger().info("Data Logger Node Initialized")

    def data_callback(self, msg):
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        data = list(msg.data)  # Convert array.array to list
        self.writer.writerow([timestamp] + data)
        self.get_logger().info(f"Logged data at {timestamp}")

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Data Logger Node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
