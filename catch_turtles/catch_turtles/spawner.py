#!/usr/bin/env python3
import rclpy
import time
from random import randint
from rclpy.node import Node
from functools import partial
from turtlesim.srv import Spawn
from turtle_interfaces.msg import TurtleList
from turtle_interfaces.srv import TurtleDeath

class SpawnerClientNode(Node):

    def __init__(self):
        super().__init__("spawner")
        self.turtle_arr = []
        self.turtle_list_publisher_= self.create_publisher(
            TurtleList, "turtle_list", 1)
        self.timer_ = self.create_timer(1.0, self.publish_turtle_list)
        self.get_logger().info("Turtle list publisher has started")

        self.idx = 1
        self.timer_ = self.create_timer(5.0, self.call_spawn_client)
        
        self.server = self.create_service(
            TurtleDeath, "turtle_death", self.register_death)
        self.get_logger().info("Server has started")

    def register_death(self, request, response):
        if request.name in self.turtle_arr: 
            self.turtle_arr.remove(request.name)
        response.turtle_arr = self.turtle_arr
        return response
    
    def call_spawn_client(self):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server SetLed")
        request = Spawn.Request()
        request.x = randint(0,1100) / 100
        request.y = randint(0,1100) / 100
        request.theta = randint(0,900) / 10
        request.name = f"target_turtle_{self.idx}"
        self.turtle_arr.append(request.name)
        self.idx += 1

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_spawner, 
                                         x=request.x, 
                                         y=request.y, 
                                         theta=request.theta))

    def callback_spawner(self, future, x, y, theta):
        try:
            response = future.result()
            coordinates = f"x: {x:.2f}, y: {y:.2f}, theta: {theta:.2f}"
            self.get_logger().info(f"Spawn Mister {response.name} at {coordinates}")
        except Exception as e:
            self.get_logger().error(f"Service failed with exception {e}")
        
    def publish_turtle_list(self):
        msg = TurtleList()
        msg.turtle_arr = self.turtle_arr
        self.turtle_list_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SpawnerClientNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()