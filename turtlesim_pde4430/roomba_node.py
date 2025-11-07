#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class DoubleSweepRoomba(Node):
    def __init__(self):
        super().__init__('double_sweep_roomba')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.timer = self.create_timer(0.1, self.loop)
        self.pose = None
        self.phase = "HORIZONTAL_SWEEP"
        self.state = "GO_TO_START"
        self.sweep_inc = 1.0
        self.row = 0
        self.col = 0
        self.max_rows = 9
        self.max_cols = 9
        self.h_start = (1.0, 10.0)
        self.v_start = (1.0, 10.0)
        self.target_x = 1.0
        self.target_y = 10.0
        self.right = True
        self.down = True
        self.get_logger().info("Double Sweep Roomba started! Horizontal + Vertical coverage.")

    def pose_cb(self, msg):
        self.pose = msg

    def loop(self):
        if not self.pose:
            return
        x, y, th = self.pose.x, self.pose.y, self.pose.theta
        msg = Twist()
        if self.phase == "HORIZONTAL_SWEEP":
            self.horizontal(x, y)
        elif self.phase == "VERTICAL_SWEEP":
            self.vertical(x, y)
        elif self.phase == "COMPLETE":
            self.pub.publish(msg)
            return
        angle_err = self.angle_err(x, y, th)
        dist_err = math.sqrt((self.target_x - x)**2 + (self.target_y - y)**2)
        msg.linear.x = min(2.0, dist_err * 1.5)
        msg.angular.z = 4.0 * angle_err
        self.pub.publish(msg)

    def horizontal(self, x, y):
        if self.state == "GO_TO_START":
            self.target_x, self.target_y = self.h_start
            if self.reached(x, y):
                self.state = "SWEEPING"
                self.get_logger().info("Horizontal sweep: Starting from top-left")
        elif self.state == "SWEEPING":
            self.target_y = self.h_start[1] - (self.row * self.sweep_inc)
            if self.right:
                self.target_x = 10.0
                if x >= 9.8:
                    if self.row >= self.max_rows:
                        self.phase = "VERTICAL_SWEEP"
                        self.state = "GO_TO_START"
                        self.row = 0
                        self.get_logger().info("Horizontal sweep complete! Starting vertical sweep.")
                    else:
                        self.row += 1
                        self.right = False
                        self.get_logger().info(f"Horizontal: Row {self.row} - Moving left")
            else:
                self.target_x = 1.0
                if x <= 1.2:
                    if self.row >= self.max_rows:
                        self.phase = "VERTICAL_SWEEP"
                        self.state = "GO_TO_START"
                        self.row = 0
                        self.get_logger().info("Horizontal sweep complete! Starting vertical sweep.")
                    else:
                        self.row += 1
                        self.right = True
                        self.get_logger().info(f"Horizontal: Row {self.row} - Moving right")

    def vertical(self, x, y):
        if self.state == "GO_TO_START":
            self.target_x, self.target_y = self.v_start
            if self.reached(x, y):
                self.state = "SWEEPING"
                self.get_logger().info("Vertical sweep: Starting from top-left")
        elif self.state == "SWEEPING":
            self.target_x = self.v_start[0] + (self.col * self.sweep_inc)
            if self.down:
                self.target_y = 1.0
                if y <= 1.2:
                    if self.col >= self.max_cols:
                        self.phase = "COMPLETE"
                        self.get_logger().info("DOUBLE COVERAGE COMPLETE! Maximum cleaning achieved!")
                    else:
                        self.col += 1
                        self.down = False
                        self.get_logger().info(f"Vertical: Column {self.col} - Moving up")
            else:
                self.target_y = 10.0
                if y >= 9.8:
                    if self.col >= self.max_cols:
                        self.phase = "COMPLETE"
                        self.get_logger().info("DOUBLE COVERAGE COMPLETE! Maximum cleaning achieved!")
                    else:
                        self.col += 1
                        self.down = True
                        self.get_logger().info(f"Vertical: Column {self.col} - Moving down")

    def reached(self, x, y):
        return math.sqrt((self.target_x - x)**2 + (self.target_y - y)**2) < 0.2

    def angle_err(self, x, y, th):
        target = math.atan2(self.target_y - y, self.target_x - x)
        return self.normalize(target - th)

    def normalize(self, angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = DoubleSweepRoomba()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
