import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        self.target_x = 8.0  # Set this to x2
        self.target_y = 8.0  # Set this to y2

        self.current_pose = None

    def pose_callback(self, msg):
        self.current_pose = msg
        self.move_to_goal()

    def move_to_goal(self):
        if self.current_pose is None:
            return
        
        velocity_msg = Twist()

        # Calculate the distance to the goal
        distance = math.sqrt((self.target_x - self.current_pose.x) ** 2 + (self.target_y - self.current_pose.y) ** 2)

        # Calculate the desired angle
        angle_to_goal = math.atan2(self.target_y - self.current_pose.y, self.target_x - self.current_pose.x)
        
        # Calculate the angular difference between the current orientation and the target orientation
        angle_diff = angle_to_goal - self.current_pose.theta
        
        # Normalize the angle
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        # Proportional controller for linear velocity
        linear_speed = 1.5 * distance

        # Proportional controller for angular velocity
        angular_speed = 6.0 * angle_diff

        # Set the velocities
        velocity_msg.linear.x = linear_speed
        velocity_msg.angular.z = angular_speed

        # Publish the command
        self.publisher.publish(velocity_msg)

        # Stop if the turtle is close enough to the goal
        if distance < 0.1:
            velocity_msg.linear.x = 0.0
            velocity_msg.angular.z = 0.0
            self.publisher.publish(velocity_msg)
            self.get_logger().info('Goal reached!')


def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
