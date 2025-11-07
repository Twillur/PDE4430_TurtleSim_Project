#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
import math, time

class FigureEight(Node):
    def __init__(self):
        super().__init__('figure_eight_node')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.move)
        self.msg = Twist()
        self.start_time = time.time()
        self.a = 2.0
        self.omega = 0.5

        client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not client.wait_for_service(1.0):
            self.get_logger().info('Waiting for teleport service...')
        req = TeleportAbsolute.Request()
        req.x = 5.5
        req.y = 5.5
        req.theta = 3*math.pi/4
        client.call_async(req)

    def move(self):
        t = time.time() - self.start_time
        x_dot = self.a*self.omega*math.cos(self.omega*t)
        y_dot = self.a*self.omega*(math.cos(self.omega*t)**2 - math.sin(self.omega*t)**2)
        d2x = -self.a*self.omega**2*math.sin(self.omega*t)
        d2y = -4*self.a*self.omega**2*math.sin(self.omega*t)*math.cos(self.omega*t)
        self.msg.linear.x = math.sqrt(x_dot**2 + y_dot**2)
        self.msg.angular.z = (x_dot*d2y - y_dot*d2x)/(x_dot**2 + y_dot**2)
        self.pub.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    node = FigureEight()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
