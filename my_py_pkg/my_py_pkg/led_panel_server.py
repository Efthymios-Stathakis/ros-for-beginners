#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLedState
from my_robot_interfaces.msg import LedStates

class LedPanelServerNode(Node):

    def __init__(self):
        super().__init__("led_panel_server")
        self.state = self.reset_state()
        self.server = self.create_service(
            SetLedState, "set_led_srv", self.callback_set_led)
        self.publisher = self.create_publisher(LedStates, "led_panel_state", 10)
        self.timer_ = self.create_timer(5, self.publish_led_state)
        self.get_logger().info("Led state server has started")
        
    def publish_led_state(self):
        msg = LedStates()
        msg.led_arr = self.state
        self.publisher.publish(msg)

    def reset_state(self):
        return [0,0,0]
    
    def callback_set_led(self, request, response):
        if request.state == True: 
            self.state[request.led] = 1
        else:
            self.state = self.reset_state()
        response.success = True
        self.get_logger().info(str(request.state) + "|" + str(request.led))
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LedPanelServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()