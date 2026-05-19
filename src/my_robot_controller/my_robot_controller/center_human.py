
from pygraphviz import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class HumanVisionNode(Node):
    def __init__(self):
        self.human_pos = self.create_publisher(Float64MultiArray, '/human_position', 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def mid_person(self, frame):
        msg = Twist()
        msg.linear.x = 0.5  # Move forward at 0.5m/s
        msg.angular.z = 0.0 # Slight turn
        self.publisher_.publish(msg)