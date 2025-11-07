#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Circle(Node):
    def __init__(self):
        super().__init__('circle_node')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move)
        self.msg = Twist()
        self.msg.linear.x = 2.0
        self.msg.angular.z = 1.0  # controls radius

    def move(self):
        self.pub.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    node = Circle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
