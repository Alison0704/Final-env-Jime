import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Float32
from cv_bridge import CvBridge
import cv2

class VisionBridge(Node):
    def __init__(self):
        super().__init__('vision_bridge')
        self.bridge = CvBridge()
        # Load the built-in OpenCV face detector
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        
        # Publishers
        self.img_pub = self.create_publisher(Image, '/camera/processed_image', 10)
        self.detect_pub = self.create_publisher(Bool, '/customer_detected', 10)
        self.face_state_pub = self.create_publisher(String, '/robot_face', 10)
        self.offset_pub = self.create_publisher(Float32, '/camera/target_offset', 10)

        # Subscriber (Assuming you started v4l2_camera_node)
        self.create_subscription(Image, '/image_raw', self.process_frame, 10)
        self.get_logger().info("Vision Bridge Online - Looking for faces...")

    def process_frame(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)

        found = len(faces) > 0
        self.detect_pub.publish(Bool(data=found))

        if found:
            # Send trigger to HTML to show camera
            self.face_state_pub.publish(String(data="SHOW_CLIENT_FEED"))
            
            # Draw box and calculate offset for ALIGNMENT
            (x, y, w, h) = faces[0]
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
            # Center of face vs center of image (-1.0 to 1.0)
            center_x = x + (w/2)
            offset = (center_x - (frame.shape[1]/2)) / (frame.shape[1]/2)
            self.offset_pub.publish(Float32(data=offset))

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

def main():
    rclpy.init()
    rclpy.spin(VisionBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
