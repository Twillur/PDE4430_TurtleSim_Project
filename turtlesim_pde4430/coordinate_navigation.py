#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
from turtlesim.msg import Pose
import math


class CoordinateNavigation(Node):
    def __init__(self):
        super().__init__('coordinate_navigation')
        
        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Subscriber to get current pose
        self.pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10
        )
        
        # Get target coordinates from user
        self.target_x = float(input("Enter target X coordinate (0-11): "))
        self.target_y = float(input("Enter target Y coordinate (0-11): "))
        
        # Current pose storage
        self.current_pose = None
        
        # Control timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f"Navigating to ({self.target_x}, {self.target_y})")

    def pose_callback(self, msg):
        self.current_pose = msg

    def control_loop(self):
        if self.current_pose is None:
            return

        msg = Twist()
        x, y, theta = self.current_pose.x, self.current_pose.y, self.current_pose.theta
        
        # Calculate distance to target
        distance_error = math.sqrt((self.target_x - x)**2 + (self.target_y - y)**2)
        
        # Calculate angle to target
        target_angle = math.atan2(self.target_y - y, self.target_x - x)
        angle_error = self.normalize_angle(target_angle - theta)
        
        # Stop when close enough to target
        if distance_error < 0.1:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info(f"🎯 Target reached! Position: ({x:.2f}, {y:.2f})")
            self.timer.cancel()  # Stop the control loop
            return
        
        # Control logic
        msg.linear.x = min(2.0, distance_error * 1.5)  # Speed proportional to distance
        msg.angular.z = 4.0 * angle_error  # Turn rate proportional to angle error
        
        self.publisher_.publish(msg)

    def normalize_angle(self, angle):
        """Normalize angle to between -pi and pi"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()