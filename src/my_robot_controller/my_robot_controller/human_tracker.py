import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs import msg
from std_msgs.msg import Float64, Float64MultiArray
from cv_bridge import CvBridge
import cv2
import time
from ultralytics import YOLO
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class HumanVisionNode(Node):
    def __init__(self):
        super().__init__('human_vision_node')
        self.model = YOLO('yolov8n.pt') 
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.tilt_pub = self.create_publisher(Float64MultiArray, '/social_tilt', 10) 

        self.force_L = 0.0
        self.force_R = 0.0
        
        self.create_subscription(
            Float64,
            '/model/grey_robot/joint/suspension_L/cmd_force',
            self.force_L_cb,
            10
        )
        self.create_subscription(
            Float64,
            '/model/grey_robot/joint/suspension_R/cmd_force',
            self.force_R_cb,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_width = 640  # Adjust to your camera resolution
        self.kp = 0.002         # Tuning constant (start small!)
        
        # ---  VARIABLES FOR SMOOTHING ---
        self.current_tilt = 0.0      # Stores the current "smoothed" value
        self.target_tilt = 0.0       # Where we WANT to go
        self.smoothing_factor = 0.2  # 0.1 = slow/smooth, 0.8 = fast/jerky
        
        # --- VARIABLES FOR EDGE FLICKER ---
        self.last_seen_time = 0.0
        self.detection_threshold = 0.5 # Seconds to wait before giving up on a human
        
        self.get_logger().info("AI Vision Node Started. Smoothing enabled.")


        self.auto_center_enabled = False  # Default to OFF

        self.toggle_sub = self.create_subscription(
            Bool,
            '/enable_centering',
            self.toggle_callback,
            10)
        
    def force_L_cb(self, msg):
        self.force_L = msg.data 
    def force_R_cb(self, msg):
        self.force_R = msg.data

    def toggle_callback(self, msg):
        self.auto_center_enabled = msg.data
        if not self.auto_center_enabled:
            self.cmd_pub.publish(Twist()) # Stop the robot when turned off



    def center_human(self, human_x):
        msg = Twist()
        
        # # Calculate error (center of image is 320 for a 640px wide image)
        # error = (self.image_width / 2) - human_x
        
        # # Deadzone: Ignore small errors to prevent "shaking"
        # if abs(error) < 20:
        #     msg.angular.z = float(0.0)
        # else:
        #     # P-Control: Rotation speed proportional to distance from center
        #     msg.angular.z = float(error * self.kp)
        
        if self.force_L > self.force_R:
            msg.angular.z = float(-0.1)  # Rotate
        elif self.force_R > self.force_L:
            msg.angular.z = float(0.1) # Rotate
        else:
            msg.angular.z = float(0.0)  # Stop rotation

        self.cmd_pub.publish(msg)


    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, classes=[0], conf=0.3, verbose=False) # Only look for "person" class, confidence

        human_detected = False
        img_width = frame.shape[1] # [1] is width, [0] is height
        
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0] # Get bounding box coordinates
                center_x = (x1 + x2) / 2
                normalized_x = (center_x - (img_width / 2)) / (img_width / 2) # normalized_x is 
                #print(f"Detected human at x={center_x:.1f} (normalized: {normalized_x:.2f})")
                
                # Update target and timestamp
                self.target_tilt = float(normalized_x)
                self.last_seen_time = time.time()
                human_detected = True

                # -- ROTATE ROBOT BASE ---
                if self.auto_center_enabled:
                    self.center_human(center_x)
                    if abs(self.force_L - self.force_R) < 0.5: 
                        self.auto_center_enabled = False
                    
                        self.cmd_pub.publish(Twist()) # Stop the robot
                        self.send_tilt_command(0.0) # Reset tilt
                        self.get_logger().info("Human centered. Auto-centering disabled.")
                # ------------------------------

                break 

        # If no human, check if we should "wait" or return to zero
        if not human_detected:
            if (time.time() - self.last_seen_time) > self.detection_threshold:
                self.target_tilt = 0.0
                if self.auto_center_enabled:
                    self.get_logger().info("No human detected. Stopping robot.")
                    self.auto_center_enabled = False
                    self.cmd_pub.publish(Twist()) # Stop the robot if no human for a while
                    self.send_tilt_command(0.0) # Reset tilt

        # --- THE GRADUAL MATH ---
        # Formula: current = current + (target - current) * factor
        self.current_tilt += (self.target_tilt - self.current_tilt) * self.smoothing_factor
        
        self.send_tilt_command(self.current_tilt)

        cv2.imshow("Robot Vision", r.plot())
        cv2.waitKey(1)

    def send_tilt_command(self, x_offset):
        sensitivity = 0.08 
        tilt_val = x_offset * sensitivity
        
        msg = Float64MultiArray()
        msg.data = [tilt_val, -tilt_val]
        self.tilt_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HumanVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()