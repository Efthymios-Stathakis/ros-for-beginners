#!/usr/bin/env python3
import rclpy
from random import randint
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberPublisher(Node):

    def __init__(self):
        super().__init__("number_pub")
        self.declare_parameter("number_to_publish", 2)
        self.declare_parameter("publish_freq", 1.0)
        self.number = self.get_parameter("number_to_publish").value
        self.freq = self.get_parameter("publish_freq").value
        self.publisher = self.create_publisher(Int64, "new_number", 10)
        self.get_logger().info("Number publisher is starting")
        self.timer_ = self.create_timer(1/self.freq, self.publish_num)

    def publish_num(self):
        msg = Int64()
        # msg.data = randint(10,100)
        msg.data = self.number
        self.publisher.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()