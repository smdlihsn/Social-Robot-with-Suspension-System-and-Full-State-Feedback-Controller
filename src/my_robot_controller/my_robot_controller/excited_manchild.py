import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class HumanVisionNode(Node):
    def __init__(self):
        super().__init__('human_vision_node')
        
        # 1. Initialize YOLO (Small version 'n' for speed)
        self.model = YOLO('yolov8n.pt') 
        self.bridge = CvBridge()
        
        # 2. Subscribe to the Gazebo Camera
        # Note: Change the topic name if your SDF uses a different one
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # 3. Publisher for the Social Tilt (The link to your FSF controller)
        self.tilt_pub = self.create_publisher(Float64MultiArray, '/social_tilt', 10)
        
        self.get_logger().info("AI Vision Node Started. Looking for humans...")

        # 
        self.timer = self.create_timer(0.05, lambda: self.dance_loop(0.0)) # 20Hz
        self.start_time = self.get_clock().now()

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format. Works with numpy versions >= 1.24.0
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Run AI Inference (stream=True for better memory handling)
        results = self.model(frame, classes=[0], conf=0.5, verbose=False) # Class 0 is 'person'

        human_detected = False
        img_width = frame.shape[1]
        
        for r in results:
            for box in r.boxes:
                # Get coordinates of the bounding box
                x1, y1, x2, y2 = box.xyxy[0]
                center_x = (x1 + x2) / 2
                
                # Calculate how far from center the human is (-1.0 to 1.0)
                normalized_x = (center_x - (img_width / 2)) / (img_width / 2)
                
                self.get_logger().info(f"Human detected at: {normalized_x:.2f}")
                
                # Send the "Social Tilt" command based on position
                self.dance_loop(0.4)
                human_detected = True
                break # Just track the first human found for now

        # If no one is seen, return to neutral height
        if not human_detected:
            self.dance_loop(0.0)

        # Optional: Pop up a window to see what the robot sees
        cv2.imshow("Robot Vision", r.plot())
        cv2.waitKey(1)

    def send_tilt_command(self, x_offset):
        # sensitivity: how much the robot leans (max 5cm)
        sensitivity = 0.08 
        tilt_val = x_offset * sensitivity
        
        msg = Float64MultiArray()
        # If human is on the right (positive x), lean right (drop right, lift left)
        msg.data = [tilt_val, -tilt_val]
        self.tilt_pub.publish(msg)


    def dance_loop(self, tilt_amplitude):
        # Time-based sine wave for a "breathing" or "dancing" effect
        now = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Example: Swaying side to side
        frequency = 5         # speed of the dance

        
        left_offset = tilt_amplitude * math.sin(frequency * now)
        right_offset = -left_offset # Inverse for a leaning effect
        
        msg = Float64MultiArray()
        msg.data = [left_offset, right_offset]
        self.tilt_pub.publish(msg)

def main():
    rclpy.init()
    node = HumanVisionNode()
    rclpy.spin(node)
    rclpy.shutdown()