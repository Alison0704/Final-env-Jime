import rclpy
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class RobotState(Enum):
    CLIENT_CHECK = 1
    CLIENT_ALIGN = 2
    CLIENT_MOVE = 3
    OPTION_SELECT = 4
    TABLE_CHECK = 5

class HostRobotLogic(Node):
    def __init__(self):
        super().__init__('host_robot_logic')
        self.bridge = CvBridge()
        self.state = RobotState.CLIENT_CHECK
        self.prev_state = None

        # --- ARUCO Setup ---
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.table_id = 42 
        self.table_detected = False
        
        # --- Camera Setup ---
        self.cap = cv2.VideoCapture(0)
        xml_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        if not os.path.exists(xml_path):
            xml_path = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'
        self.face_cascade = cv2.CascadeClassifier(xml_path)
        
        # --- State Variables ---
        self.human_detected = False
        self.target_x_offset = 0.0
        self.current_distance = 100.0  
        self.ui_confirmed = False
        self.lost_tracker_count = 0 

        # --- Publishers ---
        self.esp32_pub = self.create_publisher(String, '/esp32/motor_commands', 10)
        self.face_pub = self.create_publisher(String, '/robot_face', 10)
        self.state_pub = self.create_publisher(String, '/robot_current_state', 10)
        self.img_pub = self.create_publisher(Image, '/camera/processed_image', 10)

        # --- Subscriptions ---
        self.create_subscription(Float32, '/ultrasonic_distance', self.dist_cb, 10)
        self.create_subscription(String, '/ui_interaction', self.ui_cb, 10)

        self.timer = self.create_timer(0.1, self.main_loop)
        self.get_logger().info("JIM-E LOGIC: State Machine Active.")

    def dist_cb(self, msg): self.current_distance = msg.data
    def ui_cb(self, msg):
        if msg.data == "YES": self.ui_confirmed = True

    def send_cmd(self, cmd):
        self.esp32_pub.publish(String(data=cmd))

    def set_face(self, face_cmd):
        self.face_pub.publish(String(data=face_cmd))

    def main_loop(self):
        ret, frame = self.cap.read()
        if not ret: return

        # Face Detection Logic
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
        
        face_width = 0
        if len(faces) > 0:
            self.lost_tracker_count = 0
            self.human_detected = True
            (x, y, w, h) = faces[0]
            face_width = w
            center_x = x + (w/2)
            self.target_x_offset = (center_x - (frame.shape[1]/2)) / (frame.shape[1]/2)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        else:
            self.lost_tracker_count += 1
            if self.lost_tracker_count > 15: self.human_detected = False

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

        # --- STATE MACHINE ---
        
        if self.state == RobotState.CLIENT_CHECK:
            self.set_face("SEARCHING_EYES")
            self.send_cmd("LEFT")
            if self.human_detected: self.state = RobotState.CLIENT_ALIGN

        elif self.state == RobotState.CLIENT_ALIGN:
            if not self.human_detected: self.state = RobotState.CLIENT_CHECK
            else:
                self.set_face("SHOW_CLIENT_FEED")
                if abs(self.target_x_offset) < 0.2:
                    self.send_cmd("STOP")
                    self.state = RobotState.CLIENT_MOVE
                else:
                    self.send_cmd("LEFT" if self.target_x_offset < 0 else "RIGHT")

        elif self.state == RobotState.CLIENT_MOVE:
            if not self.human_detected: 
                self.state = RobotState.CLIENT_CHECK
            else:
                self.set_face("SHOW_CLIENT_FEED")
                # TRIGGER: 50cm OR face is wide enough (close to camera)
                if self.current_distance <= 50.0 or face_width > 180:
                    self.send_cmd("STOP")
                    self.state = RobotState.OPTION_SELECT
                else:
                    self.send_cmd("FORWARD")

        elif self.state == RobotState.OPTION_SELECT:
            self.set_face("SHOW_UI_BUTTONS")
            self.send_cmd("STOP")
            if self.ui_confirmed:
                self.ui_confirmed = False
                self.state = RobotState.TABLE_CHECK

        elif self.state == RobotState.TABLE_CHECK:
            self.set_face("SEARCHING_EYES")
            self.send_cmd("RIGHT")

        # Publish state for debugging
        if self.state != self.prev_state:
            self.state_pub.publish(String(data=self.state.name))
            self.prev_state = self.state

def main(args=None):
    rclpy.init(args=args)
    node = HostRobotLogic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
