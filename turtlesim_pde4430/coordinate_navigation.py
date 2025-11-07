#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class CoordinateNavigation(Node):
    def __init__(self):
        super().__init__('coordinate_navigation')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose = None
        self.sub = self.create_subscription(Pose, '/turtle1/pose', lambda msg: setattr(self,'pose',msg), 10)
        self.target_x = float(input("Enter target X coordinate (0-11): "))
        self.target_y = float(input("Enter target Y coordinate (0-11): "))
        self.timer = self.create_timer(0.1, self.control)
        self.get_logger().info(f"Navigating to ({self.target_x}, {self.target_y})")

    def control(self):
        if not self.pose:
            return
        msg = Twist()
        x, y, theta = self.pose.x, self.pose.y, self.pose.theta
        dx, dy = self.target_x - x, self.target_y - y
        dist = math.sqrt(dx**2 + dy**2)
        angle_err = self.normalize(math.atan2(dy, dx) - theta)
        if dist < 0.1:
            msg.linear.x = msg.angular.z = 0.0
            self.pub.publish(msg)
            self.get_logger().info(f"🎯 Target reached! Position: ({x:.2f}, {y:.2f})")
            self.timer.cancel()
            return
        msg.linear.x = min(2.0, dist*1.5)
        msg.angular.z = 4.0 * angle_err
        self.pub.publish(msg)

    def normalize(self, angle):
        while angle > math.pi: angle -= 2*math.pi
        while angle < -math.pi: angle += 2*math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = CoordinateNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
