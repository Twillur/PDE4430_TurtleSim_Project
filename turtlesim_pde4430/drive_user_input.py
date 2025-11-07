#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleUserDrive(Node):
    def __init__(self):
        super().__init__('turtle_user_drive')
        self.pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.linear = float(input("Enter linear speed: "))
        self.angular = float(input("Enter angular speed: "))
        self.timer = self.create_timer(0.1, self.publish_cmd)

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleUserDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
