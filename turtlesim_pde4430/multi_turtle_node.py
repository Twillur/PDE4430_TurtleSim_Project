#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, Spawn, Kill
from turtlesim.msg import Pose
import math


class FullMapCleaner(Node):
    def __init__(self):
        super().__init__('full_map_cleaner')

        # ROS publishers/subscribers
        self.publisher1_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.publisher2_ = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.publisher3_ = self.create_publisher(Twist, '/turtle3/cmd_vel', 10)
        self.publisher4_ = self.create_publisher(Twist, '/turtle4/cmd_vel', 10)
        
        self.pose_subscriber1 = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback1, 10
        )
        self.pose_subscriber2 = self.create_subscription(
            Pose, '/turtle2/pose', self.pose_callback2, 10
        )
        self.pose_subscriber3 = self.create_subscription(
            Pose, '/turtle3/pose', self.pose_callback3, 10
        )
        self.pose_subscriber4 = self.create_subscription(
            Pose, '/turtle4/pose', self.pose_callback4, 10
        )

        # Control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)

        # State for turtle1 (top-right quadrant)
        self.current_pose1 = None
        self.moving_right1 = True
        self.current_row1 = 0
        self.complete1 = False

        # Sweep limits for turtle1 (top-right quadrant)
        self.x_min1 = 5.5
        self.x_max1 = 11.0
        self.y_max1 = 11.0
        self.y_min1 = 5.5
        self.sweep_increment1 = 0.7
        self.max_rows1 = int((self.y_max1 - self.y_min1) / self.sweep_increment1)

        # State for turtle2 (top-left quadrant)  
        self.current_pose2 = None
        self.moving_up2 = True
        self.current_row2 = 0
        self.complete2 = False

        # Sweep limits for turtle2 (top-left quadrant)
        self.x_min2 = 0.0
        self.x_max2 = 5.5
        self.y_max2 = 11.0
        self.y_min2 = 5.5
        self.sweep_increment2 = 0.7
        self.max_rows2 = int((self.x_max2 - self.x_min2) / self.sweep_increment2)

        # State for turtle3 (bottom-right quadrant)
        self.current_pose3 = None
        self.moving_right3 = True
        self.current_row3 = 0
        self.complete3 = False

        # Sweep limits for turtle3 (bottom-right quadrant)
        self.x_min3 = 5.5
        self.x_max3 = 11.0
        self.y_max3 = 5.5
        self.y_min3 = 0.0
        self.sweep_increment3 = 0.7
        self.max_rows3 = int((self.y_max3 - self.y_min3) / self.sweep_increment3)

        # State for turtle4 (bottom-left quadrant)
        self.current_pose4 = None
        self.moving_up4 = True
        self.current_row4 = 0
        self.complete4 = False

        # Sweep limits for turtle4 (bottom-left quadrant)
        self.x_min4 = 0.0
        self.x_max4 = 5.5
        self.y_max4 = 5.5
        self.y_min4 = 0.0
        self.sweep_increment4 = 0.7
        self.max_rows4 = int((self.x_max4 - self.x_min4) / self.sweep_increment4)

        # Initialize turtles
        self.spawn_additional_turtles()

    def spawn_additional_turtles(self):
        """Spawn additional turtles and position all turtles."""
        # Kill default turtle if needed and spawn all turtles
        self.kill_default_turtle()
        
        # Spawn all turtles
        self.spawn_turtle('turtle1', 5.5, 5.5, 0.0)  # Top-right - face right
        self.spawn_turtle('turtle2', 5.5, 5.5, math.pi/2)  # Top-left - face up
        self.spawn_turtle('turtle3', 5.5, 5.5, 0.0)  # Bottom-right - face right  
        self.spawn_turtle('turtle4', 5.5, 5.5, math.pi/2)  # Bottom-left - face up

        # Position turtles in their respective quadrants
        self.teleport_turtle('turtle1', self.x_min1, self.y_max1, 0.0)  # Top-right
        self.teleport_turtle('turtle2', 5.5, self.y_min2, math.pi/2)    # Top-left
        self.teleport_turtle('turtle3', self.x_max3, self.y_min3, math.pi)  # Bottom-right
        self.teleport_turtle('turtle4', 0.0, self.y_min4, math.pi/2)    # Bottom-left

        self.get_logger().info("FULL MAP CLEANER STARTED!")
        self.get_logger().info("Turtle1: Cleaning TOP-RIGHT quadrant (horizontal sweep)")
        self.get_logger().info("Turtle2: Cleaning TOP-LEFT quadrant (vertical sweep)")
        self.get_logger().info("Turtle3: Cleaning BOTTOM-RIGHT quadrant (horizontal sweep)")
        self.get_logger().info("Turtle4: Cleaning BOTTOM-LEFT quadrant (vertical sweep)")

    def kill_default_turtle(self):
        """Kill the default turtle to avoid conflicts."""
        client = self.create_client(Kill, '/kill')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for kill service...')
        req = Kill.Request()
        req.name = 'turtle1'
        client.call_async(req)

    def spawn_turtle(self, name, x, y, theta):
        """Spawn a turtle at specified position."""
        spawn_client = self.create_client(Spawn, '/spawn')
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        
        spawn_req = Spawn.Request()
        spawn_req.x = x
        spawn_req.y = y
        spawn_req.theta = theta
        spawn_req.name = name
        spawn_client.call_async(spawn_req)

    def teleport_turtle(self, turtle_name, x, y, theta):
        """Teleport turtle to specified position."""
        client = self.create_client(TeleportAbsolute, f'/{turtle_name}/teleport_absolute')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {turtle_name} teleport service...')
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        client.call_async(req)

    def pose_callback1(self, msg):
        self.current_pose1 = msg

    def pose_callback2(self, msg):
        self.current_pose2 = msg

    def pose_callback3(self, msg):
        self.current_pose3 = msg

    def pose_callback4(self, msg):
        self.current_pose4 = msg

    def control_loop(self):
        if (self.current_pose1 is None or self.current_pose2 is None or 
            self.current_pose3 is None or self.current_pose4 is None):
            return

        # Control all turtles
        if not self.complete1:
            self.control_turtle1()
        if not self.complete2:
            self.control_turtle2()
        if not self.complete3:
            self.control_turtle3()
        if not self.complete4:
            self.control_turtle4()

        # Check if all complete
        if all([self.complete1, self.complete2, self.complete3, self.complete4]):
            self.get_logger().info("🎉 ALL QUADRANTS COMPLETELY CLEANED! 🎉")
            self.timer.cancel()

    def control_turtle1(self):
        """Control turtle1 for top-right quadrant (horizontal sweep)."""
        msg = Twist()
        x, y, theta = self.current_pose1.x, self.current_pose1.y, self.current_pose1.theta

        # Determine target for current sweep
        if self.moving_right1:
            target_x = self.x_max1
            target_y = self.y_max1 - (self.current_row1 * self.sweep_increment1)
            if x >= self.x_max1 - 0.2:
                if self.current_row1 >= self.max_rows1:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.publisher1_.publish(msg)
                    self.complete1 = True
                    self.get_logger().info("✅ Turtle1: TOP-RIGHT QUADRANT COMPLETE!")
                    return
                self.current_row1 += 1
                self.moving_right1 = False
        else:
            target_x = self.x_min1
            target_y = self.y_max1 - (self.current_row1 * self.sweep_increment1)
            if x <= self.x_min1 + 0.2:
                if self.current_row1 >= self.max_rows1:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.publisher1_.publish(msg)
                    self.complete1 = True
                    self.get_logger().info("✅ Turtle1: TOP-RIGHT QUADRANT COMPLETE!")
                    return
                self.current_row1 += 1
                self.moving_right1 = True

        # Smooth control
        angle_error = self.calculate_angle_error(x, y, theta, target_x, target_y)
        distance_error = math.sqrt((target_x - x)**2 + (target_y - y)**2)

        msg.linear.x = min(2.0, distance_error * 1.5)
        msg.angular.z = 4.0 * angle_error
        self.publisher1_.publish(msg)

    def control_turtle2(self):
        """Control turtle2 for top-left quadrant (vertical sweep)."""
        msg = Twist()
        x, y, theta = self.current_pose2.x, self.current_pose2.y, self.current_pose2.theta

        # Determine target for current sweep (vertical movement)
        if self.moving_up2:
            target_x = self.x_min2 + (self.current_row2 * self.sweep_increment2)
            target_y = self.y_max2
            if y >= self.y_max2 - 0.2:
                if self.current_row2 >= self.max_rows2:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.publisher2_.publish(msg)
                    self.complete2 = True
                    self.get_logger().info("✅ Turtle2: TOP-LEFT QUADRANT COMPLETE!")
                    return
                self.current_row2 += 1
                self.moving_up2 = False
        else:
            target_x = self.x_min2 + (self.current_row2 * self.sweep_increment2)
            target_y = self.y_min2
            if y <= self.y_min2 + 0.2:
                if self.current_row2 >= self.max_rows2:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.publisher2_.publish(msg)
                    self.complete2 = True
                    self.get_logger().info("✅ Turtle2: TOP-LEFT QUADRANT COMPLETE!")
                    return
                self.current_row2 += 1
                self.moving_up2 = True

        # Smooth control
        angle_error = self.calculate_angle_error(x, y, theta, target_x, target_y)
        distance_error = math.sqrt((target_x - x)**2 + (target_y - y)**2)

        msg.linear.x = min(2.0, distance_error * 1.5)
        msg.angular.z = 4.0 * angle_error
        self.publisher2_.publish(msg)

    def control_turtle3(self):
        """Control turtle3 for bottom-right quadrant (horizontal sweep)."""
        msg = Twist()
        x, y, theta = self.current_pose3.x, self.current_pose3.y, self.current_pose3.theta

        # Determine target for current sweep
        if self.moving_right3:
            target_x = self.x_max3
            target_y = self.y_min3 + (self.current_row3 * self.sweep_increment3)
            if x >= self.x_max3 - 0.2:
                if self.current_row3 >= self.max_rows3:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.publisher3_.publish(msg)
                    self.complete3 = True
                    self.get_logger().info("✅ Turtle3: BOTTOM-RIGHT QUADRANT COMPLETE!")
                    return
                self.current_row3 += 1
                self.moving_right3 = False
        else:
            target_x = self.x_min3
            target_y = self.y_min3 + (self.current_row3 * self.sweep_increment3)
            if x <= self.x_min3 + 0.2:
                if self.current_row3 >= self.max_rows3:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.publisher3_.publish(msg)
                    self.complete3 = True
                    self.get_logger().info("✅ Turtle3: BOTTOM-RIGHT QUADRANT COMPLETE!")
                    return
                self.current_row3 += 1
                self.moving_right3 = True

        # Smooth control
        angle_error = self.calculate_angle_error(x, y, theta, target_x, target_y)
        distance_error = math.sqrt((target_x - x)**2 + (target_y - y)**2)

        msg.linear.x = min(2.0, distance_error * 1.5)
        msg.angular.z = 4.0 * angle_error
        self.publisher3_.publish(msg)

    def control_turtle4(self):
        """Control turtle4 for bottom-left quadrant (vertical sweep)."""
        msg = Twist()
        x, y, theta = self.current_pose4.x, self.current_pose4.y, self.current_pose4.theta

        # Determine target for current sweep (vertical movement)
        if self.moving_up4:
            target_x = self.x_min4 + (self.current_row4 * self.sweep_increment4)
            target_y = self.y_max4
            if y >= self.y_max4 - 0.2:
                if self.current_row4 >= self.max_rows4:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.publisher4_.publish(msg)
                    # Return to origin
                    self.teleport_turtle('turtle4', 5.0, 5.0, math.pi/2)
                    self.complete4 = True
                    self.get_logger().info("✅ Turtle4: BOTTOM-LEFT QUADRANT COMPLETE!")
                    return
                self.current_row4 += 1
                self.moving_up4 = False
        else:
            target_x = self.x_min4 + (self.current_row4 * self.sweep_increment4)
            target_y = self.y_min4
            if y <= self.y_min4 + 0.2:
                if self.current_row4 >= self.max_rows4:
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.publisher4_.publish(msg)
                    # Return to origin
                    self.teleport_turtle('turtle4', 5.0, 5.0, math.pi/2)
                    self.complete4 = True
                    self.get_logger().info("✅ Turtle4: BOTTOM-LEFT QUADRANT COMPLETE!")
                    return
                self.current_row4 += 1
                self.moving_up4 = True

        # Smooth control
        angle_error = self.calculate_angle_error(x, y, theta, target_x, target_y)
        distance_error = math.sqrt((target_x - x)**2 + (target_y - y)**2)

        msg.linear.x = min(2.0, distance_error * 1.5)
        msg.angular.z = 4.0 * angle_error
        self.publisher4_.publish(msg)

    def calculate_angle_error(self, x, y, theta, target_x, target_y):
        target_angle = math.atan2(target_y - y, target_x - x)
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
    cleaner = FullMapCleaner()
    rclpy.spin(cleaner)
    cleaner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()