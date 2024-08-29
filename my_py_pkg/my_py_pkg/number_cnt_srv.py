#!/usr/bin/env python3
import rclpy
from random import randint
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class ResetNumberCounter(Node):

    def __init__(self):
        super().__init__("number_counter_srv")
        self.get_logger().info("Number counter is starting")
        self.cnt = 0
        self.subscriber = self.create_subscription(
            Int64, "new_number", self.count_num, 10)
        self.publisher = self.create_publisher(Int64, "cnt_number", 10)
        self.server = self.create_service(
            SetBool, "reset_cnt", self.callback_reset_cnt)
        self.get_logger().info("Server has started")

    def count_num(self, msg):
        self.get_logger().info(str(msg.data))
        self.cnt = self.cnt + int(msg.data)

        pub_msg = Int64()
        pub_msg.data = self.cnt
        self.publisher.publish(pub_msg)
        
    def callback_reset_cnt(self, request, response):
        if request.data: 
            self.cnt = 0
            str_msg = "Reseting counter"
        else:
            str_msg = "Counter left intact"
        self.get_logger().info(str_msg)
        response.success = request.data
        response.message = str_msg
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = ResetNumberCounter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()