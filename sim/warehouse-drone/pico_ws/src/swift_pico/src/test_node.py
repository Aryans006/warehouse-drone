#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class MultiArrayReader(Node):
    def __init__(self):
        super().__init__('multiarray_reader')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/random_points',  # replace with your topic name
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # Access the data directly
        data = msg.data
        print("Received data:", data)

        # For example, accessing the first element
        if data:
            print("First element:", data[0])

def main(args=None):
    rclpy.init(args=args)
    node = MultiArrayReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
