#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
import math
import time

class FigureEight(Node):
    def __init__(self):
        super().__init__('figure_eight_node')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.move)
        self.msg = Twist()
        self.start_time = time.time()
        
        # Parameters for the figure-eight
        self.a = 2.0  # Size parameter
        self.omega = 0.5  # Angular frequency (controls speed)

        # Teleport turtle to starting position
        client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service...')
        req = TeleportAbsolute.Request()
        req.x = 5.5
        req.y = 5.5
        req.theta = 3*math.pi/4  # CHANGED: Start facing UP (90 degrees) instead of right (0 degrees)
        client.call_async(req)

    def move(self):
        t = time.time() - self.start_time
        
        # Parametric equations for a figure-eight (lemniscate)
        # x = a * sin(ωt)
        # y = a * sin(ωt) * cos(ωt)
        
        # Calculate desired position
        x_desired = 5.5 + self.a * math.sin(self.omega * t)
        y_desired = 5.5 + self.a * math.sin(self.omega * t) * math.cos(self.omega * t)
        
        # Calculate derivatives for velocity
        dx_dt = self.a * self.omega * math.cos(self.omega * t)
        dy_dt = self.a * self.omega * (math.cos(self.omega * t)**2 - math.sin(self.omega * t)**2)
        
        # Calculate linear and angular velocity
        linear_vel = math.sqrt(dx_dt**2 + dy_dt**2)
        
        # Calculate desired heading (angle)
        desired_theta = math.atan2(dy_dt, dx_dt)
        
        # For angular velocity, we can use the derivative of the desired heading
        # dθ/dt = (dx*d²y/dt² - dy*d²x/dt²) / (dx² + dy²)
        d2x_dt2 = -self.a * self.omega**2 * math.sin(self.omega * t)
        d2y_dt2 = -4 * self.a * self.omega**2 * math.sin(self.omega * t) * math.cos(self.omega * t)
        
        angular_vel = (dx_dt * d2y_dt2 - dy_dt * d2x_dt2) / (dx_dt**2 + dy_dt**2)
        
        # Set the velocities
        self.msg.linear.x = linear_vel
        self.msg.angular.z = angular_vel
        
        self.pub.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    node = FigureEight()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()