#!/usr/bin/env python3
import rclpy
from random import randint
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberCounter(Node):

    def __init__(self):
        super().__init__("number_counter")
        self.get_logger().info("Number counter is starting")
        self.cnt = 0
        self.subscriber = self.create_subscription(
            Int64, "new_number", self.count_num, 10)
        self.publisher = self.create_publisher(Int64, "cnt_number", 10)

    def count_num(self, msg):
        self.get_logger().info(str(msg.data))
        self.cnt = self.cnt + int(msg.data)

        pub_msg = Int64()
        pub_msg.data = self.cnt
        self.publisher.publish(pub_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = NumberCounter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()