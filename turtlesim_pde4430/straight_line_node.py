import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class StraightLine(Node):
    def __init__(self):
        super().__init__('straight_line_node')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, lambda: self.pub.publish(Twist(linear=Twist.Linear(x=2.0), angular=Twist.Angular(z=0.0))))

def main():
    rclpy.init()
    rclpy.spin(StraightLine())
    rclpy.shutdown()

if name == '__main__':
    main()
