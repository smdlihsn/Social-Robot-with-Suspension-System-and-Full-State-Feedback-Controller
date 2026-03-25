import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DummyDriver(Node):
    def __init__(self):
        super().__init__('dummy_driver')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_robot)

    def move_robot(self):
        msg = Twist()
        msg.linear.x = 0.5  # Move forward at 0.5m/s
        msg.angular.z = 0.0 # Slight turn
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = DummyDriver()
    rclpy.spin(node)