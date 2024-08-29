#!/usr/bin/env python3
import rclpy
import time
from random import randint
from rclpy.node import Node
from functools import partial
from my_robot_interfaces.srv import SetLedState

class SetLedClientNode(Node):

    def __init__(self):
        super().__init__("led_panel_client")
        while True:
            led = randint(0,2)
            state = bool(randint(0,1))
            self.call_set_led_state_client(led, state)
            time.sleep(randint(3,7))
        # self.call_set_led_state_client(0, True)
        # time.sleep(randint(3,7))
        # self.call_set_led_state_client(2, True)
        # time.sleep(randint(3,7))
        # self.call_set_led_state_client(12, False)

    def call_set_led_state_client(self, led, state):
        client = self.create_client(SetLedState, "set_led_srv")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server SetLed")
        request = SetLedState.Request()
        request.state = state
        request.led = led

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_led_state, led=led, state=state))

    def callback_set_led_state(self, future, led, state):
        try:
            response = future.result()
            self.get_logger().info(str(led) + "|" + str(state) + "|" + str(response.success))
        except Exception as e:
            self.get_logger().error(f"Service failed with exception {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SetLedClientNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()