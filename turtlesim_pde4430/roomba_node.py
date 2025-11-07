#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class DoubleSweepRoomba(Node):
    def __init__(self):
        super().__init__('double_sweep_roomba')
        
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10
        )
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Double sweep pattern
        self.current_pose = None
        self.phase = "HORIZONTAL_SWEEP"  # HORIZONTAL_SWEEP, VERTICAL_SWEEP, COMPLETE
        self.state = "GO_TO_START"
        
        # Sweep parameters
        self.sweep_increment = 1.0
        self.current_row = 0
        self.current_col = 0
        self.max_rows = 9
        self.max_cols = 9
        
        # Start positions for each phase
        self.horizontal_start = (1.0, 10.0)  # Top-left
        self.vertical_start = (1.0, 10.0)    # Top-left
        
        self.target_x = 1.0
        self.target_y = 10.0
        
        self.moving_right = True
        self.moving_down = True
        
        self.get_logger().info("Double Sweep Roomba started! Horizontal + Vertical coverage.")

    def pose_callback(self, msg):
        self.current_pose = msg

    def control_loop(self):
        if self.current_pose is None:
            return
            
        msg = Twist()
        x, y, theta = self.current_pose.x, self.current_pose.y, self.current_pose.theta
        
        if self.phase == "HORIZONTAL_SWEEP":
            self.horizontal_sweep(x, y, msg)
        elif self.phase == "VERTICAL_SWEEP":
            self.vertical_sweep(x, y, msg)
        elif self.phase == "COMPLETE":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            return

        # Move toward target
        angle_error = self.calculate_angle_error(x, y, theta)
        distance_error = math.sqrt((self.target_x - x)**2 + (self.target_y - y)**2)
        
        msg.linear.x = min(2.0, distance_error * 1.5)
        msg.angular.z = 4.0 * angle_error
        
        self.publisher_.publish(msg)

    def horizontal_sweep(self, x, y, msg):
        if self.state == "GO_TO_START":
            self.target_x, self.target_y = self.horizontal_start
            if self.reached_target(x, y):
                self.state = "SWEEPING"
                self.get_logger().info("Horizontal sweep: Starting from top-left")
        
        elif self.state == "SWEEPING":
            if self.moving_right:
                # Move right to right wall
                self.target_x = 10.0
                self.target_y = self.horizontal_start[1] - (self.current_row * self.sweep_increment)
                
                if x >= 9.8:
                    if self.current_row >= self.max_rows:
                        # Horizontal sweep complete, start vertical
                        self.phase = "VERTICAL_SWEEP"
                        self.state = "GO_TO_START"
                        self.current_row = 0
                        self.get_logger().info("Horizontal sweep complete! Starting vertical sweep.")
                    else:
                        self.current_row += 1
                        self.moving_right = False
                        self.get_logger().info(f"Horizontal: Row {self.current_row} - Moving left")
            
            else:
                # Move left to left wall
                self.target_x = 1.0
                self.target_y = self.horizontal_start[1] - (self.current_row * self.sweep_increment)
                
                if x <= 1.2:
                    if self.current_row >= self.max_rows:
                        self.phase = "VERTICAL_SWEEP"
                        self.state = "GO_TO_START"
                        self.current_row = 0
                        self.get_logger().info("Horizontal sweep complete! Starting vertical sweep.")
                    else:
                        self.current_row += 1
                        self.moving_right = True
                        self.get_logger().info(f"Horizontal: Row {self.current_row} - Moving right")

    def vertical_sweep(self, x, y, msg):
        if self.state == "GO_TO_START":
            self.target_x, self.target_y = self.vertical_start
            if self.reached_target(x, y):
                self.state = "SWEEPING"
                self.get_logger().info("Vertical sweep: Starting from top-left")
        
        elif self.state == "SWEEPING":
            if self.moving_down:
                # Move down to bottom wall
                self.target_x = self.vertical_start[0] + (self.current_col * self.sweep_increment)
                self.target_y = 1.0
                
                if y <= 1.2:
                    if self.current_col >= self.max_cols:
                        # Both sweeps complete!
                        self.phase = "COMPLETE"
                        self.get_logger().info("DOUBLE COVERAGE COMPLETE! Maximum cleaning achieved!")
                    else:
                        self.current_col += 1
                        self.moving_down = False
                        self.get_logger().info(f"Vertical: Column {self.current_col} - Moving up")
            
            else:
                # Move up to top wall
                self.target_x = self.vertical_start[0] + (self.current_col * self.sweep_increment)
                self.target_y = 10.0
                
                if y >= 9.8:
                    if self.current_col >= self.max_cols:
                        self.phase = "COMPLETE"
                        self.get_logger().info("DOUBLE COVERAGE COMPLETE! Maximum cleaning achieved!")
                    else:
                        self.current_col += 1
                        self.moving_down = True
                        self.get_logger().info(f"Vertical: Column {self.current_col} - Moving down")

    def reached_target(self, x, y):
        return math.sqrt((self.target_x - x)**2 + (self.target_y - y)**2) < 0.2

    def calculate_angle_error(self, x, y, theta):
        target_angle = math.atan2(self.target_y - y, self.target_x - x)
        angle_error = self.normalize_angle(target_angle - theta)
        return angle_error

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    roomba = DoubleSweepRoomba()
    rclpy.spin(roomba)
    roomba.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()