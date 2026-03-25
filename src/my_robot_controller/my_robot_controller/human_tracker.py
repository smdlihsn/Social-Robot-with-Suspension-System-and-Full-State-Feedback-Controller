import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import time
from ultralytics import YOLO

class HumanVisionNode(Node):
    def __init__(self):
        super().__init__('human_vision_node')
        self.model = YOLO('yolov8n.pt') 
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.tilt_pub = self.create_publisher(Float64MultiArray, '/social_tilt', 10)
        
        # ---  VARIABLES FOR SMOOTHING ---
        self.current_tilt = 0.0      # Stores the current "smoothed" value
        self.target_tilt = 0.0       # Where we WANT to go
        self.smoothing_factor = 0.1  # 0.1 = slow/smooth, 0.8 = fast/jerky
        
        # --- VARIABLES FOR EDGE FLICKER ---
        self.last_seen_time = 0.0
        self.detection_threshold = 0.5 # Seconds to wait before giving up on a human
        
        self.get_logger().info("AI Vision Node Started. Smoothing enabled.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, classes=[0], conf=0.5, verbose=False) # Only detect humans (class 0)

        human_detected = False
        img_width = frame.shape[1] # [1] is width, [0] is height
        
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0] # Get bounding box coordinates
                center_x = (x1 + x2) / 2
                normalized_x = (center_x - (img_width / 2)) / (img_width / 2) #
                
                # Update target and timestamp
                self.target_tilt = float(normalized_x)
                self.last_seen_time = time.time()
                human_detected = True
                break 

        # If no human, check if we should "wait" or return to zero
        if not human_detected:
            if (time.time() - self.last_seen_time) > self.detection_threshold:
                self.target_tilt = 0.0

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