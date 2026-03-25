import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class SocialIntentNode(Node):
    def __init__(self):
        super().__init__('social_intent_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/social_tilt', 10)
        self.timer = self.create_timer(0.05, self.dance_loop) # 20Hz
        self.start_time = self.get_clock().now()

    def dance_loop(self):
        # Time-based sine wave for a "breathing" or "dancing" effect
        now = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Example: Swaying side to side
        tilt_amplitude = 0.04  # 4cm
        frequency = 5         # speed of the dance

        
        left_offset = tilt_amplitude * math.sin(frequency * now)
        right_offset = -left_offset # Inverse for a leaning effect
        
        msg = Float64MultiArray()
        msg.data = [left_offset, right_offset]
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = SocialIntentNode()
    rclpy.spin(node)
    rclpy.shutdown()