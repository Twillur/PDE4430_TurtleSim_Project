#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleUserDrive(Node):
    def __init__(self):
        super().__init__('turtle_user_drive')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # Ask user for input once
        self.linear_speed = float(input("Enter linear speed: "))
        self.angular_speed = float(input("Enter angular speed: "))

        # Timer to publish commands at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_cmd)

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleUserDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
