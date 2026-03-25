import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import numpy as np

class SquareNavigator(Node):
    def __init__(self):
        super().__init__('square_navigator')
        
        # 1. Define the Square Waypoints (x, y)
        self.waypoints = [
            (2.0, 0.0),  # Point 1
            (2.0, 2.0),  # Point 2
            (0.0, 2.0),  # Point 3
            (0.0, 0.0)   # Back to start
        ]
        self.current_waypoint_idx = 0
        
        # 2. State Variables
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0
        
        # 3. Subscriptions & Publishers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 4. Control Timer (20Hz)
        self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg):
        # Update current position
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        
        # Convert Quaternion to Yaw (Euler Angle)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.curr_yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if self.current_waypoint_idx >= len(self.waypoints):
            self.current_waypoint_idx = 0 # Loop the square forever
            
        goal_x, goal_y = self.waypoints[self.current_waypoint_idx]
        
        # Calculate Errors
        dx = goal_x - self.curr_x
        dy = goal_y - self.curr_y
        dist = math.sqrt(dx**2 + dy**2)
        
        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.curr_yaw
        
        # Normalize angle error to [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        msg = Twist()

        # LOGIC: 
        # 1. If we are far from the target angle, rotate in place first
        if abs(angle_error) > 0.2:
            msg.angular.z = 0.5 if angle_error > 0 else -0.5
            msg.linear.x = 0.0
        # 2. If aligned, drive forward
        elif dist > 0.1:
            msg.linear.x = 0.3
            msg.angular.z = 0.5 * angle_error # Small corrections while driving
        # 3. If reached waypoint, move to next
        else:
            self.get_logger().info(f"Reached Waypoint {self.current_waypoint_idx}!")
            self.current_waypoint_idx += 1
            
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = SquareNavigator()
    rclpy.spin(node)
    rclpy.shutdown()