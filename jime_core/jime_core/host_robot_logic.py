import rclpy
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

class RobotState(Enum):
    #CLIENT
    CLIENT_CHECK = 1 
    CLIENT_ALIGN = 2
    CLIENT_MOVE = 3
    CLIENT_OPTION_SELECT = 4
    #TABLE
    TABLE_CHECK = 5
    TABLE_ALIGN = 6
    TABLE_MOVE = 7
    TABLE_OPTION_SELECT = 8
    #HOME
    HOME_CHECK = 9
    HOME_ALIGN = 10
    HOME_MOVE = 11

class HostRobotLogic(Node):
    def __init__(self):
        super().__init__('host_robot_logic')
        self.bridge = CvBridge()
        self.state = RobotState.CLIENT_CHECK
        self.prev_state = None

        # --- ARUCO Setup (Version Compatible) ---
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        try:
            # Newer OpenCV (4.7+)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.use_new_aruco = True
        except AttributeError:
            # Older OpenCV (< 4.7)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.use_new_aruco = False
        
        # --- Camera Setup ---
        self.cap = cv2.VideoCapture(0)
        xml_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        self.face_cascade = cv2.CascadeClassifier(xml_path)
        
        # --- State Variables ---
        self.ui_confirmed = False
        self.lost_tracker_count = 0 
        
        self.human_detected = False
        self.human_x_offset = 0.0
        self.current_distance = 100.0  

        self.table_id = 42 
        self.table_detected = False
        self.table_x_offset = 0.0
        self.table_pixel_width = 0.0

        self.home_id = 25
        self.home_detected = False
        self.home_x_offset = 0.0
        self.home_pixel_width = 0.0

        # --- Publishers ---
        self.face_pub = self.create_publisher(String, '/robot_face', 10)
        self.state_pub = self.create_publisher(String, '/robot_current_state', 10)
        self.img_pub = self.create_publisher(Image, '/camera_processed_image', 10)
        self.cmd_publisher = self.create_publisher(String, 'robot_cmd', 10)

        # --- Subscriptions ---
        self.create_subscription(Float32, '/ultrasonic_distance', self.dist_cb, 10)
        self.create_subscription(String, '/ui_interaction', self.ui_cb, 10)

        self.timer = self.create_timer(0.1, self.main_loop)
        self.get_logger().info("JIM-E LOGIC: State Machine Active.")

    def dist_cb(self, msg): 
        self.current_distance = msg.data
        
    def ui_cb(self, msg):
        if msg.data == "YES": self.ui_confirmed = True

    def send_cmd(self, cmd):
        self.cmd_publisher.publish(String(data=cmd))

    def set_face(self, face_cmd):
        self.face_pub.publish(String(data=face_cmd))

    def main_loop(self):
        ret, frame = self.cap.read()
        if not ret: return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Face Detection Logic
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
        face_width = 0
        if len(faces) > 0:
            self.lost_tracker_count = 0
            self.human_detected = True
            (x, y, w, h) = faces[0]
            face_width = w
            center_x = x + (w/2)
            self.human_x_offset = (center_x - (frame.shape[1]/2)) / (frame.shape[1]/2)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        else:
            self.lost_tracker_count += 1
            if self.lost_tracker_count > 15: self.human_detected = False

        # ArUco Detection Logic
        corners, ids, rejected = self.detector.detectMarkers(frame)
        self.table_detected = False
        self.home_detected = False

        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i][0]
                
                # Calculate common values
                c = corners[i][0]

                width_top = np.linalg.norm(c[0] - c[1])
                width_bottom = np.linalg.norm(c[3] - c[2])
                marker_width = (width_top + width_bottom) / 2
                
                marker_center_x = np.mean(c[:, 0])
                # Normalize offset: -1.0 (left) to 1.0 (right)
                x_offset = (marker_center_x - (frame.shape[1]/2)) / (frame.shape[1]/2)

                # --- TABLE TARGET (ID 42) ---
                if marker_id == self.table_id:
                    self.table_detected = True
                    self.table_x_offset = x_offset
                    self.table_pixel_width = marker_width
                    
                    # Visual feedback for Table
                    cv2.putText(frame, "TABLE TARGET", (int(c[0][0]), int(c[0][1]) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                # --- HOME TARGET (ID 25) ---
                elif marker_id == self.home_id:
                    self.home_detected = True
                    self.home_x_offset = x_offset
                    self.home_pixel_width = marker_width
                    
                    # Visual feedback for Home
                    cv2.putText(frame, "HOME STATION", (int(c[0][0]), int(c[0][1]) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # Stream the processed frame
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        
        # ============================================================
        # ============= S T A T E   M A C H I N E S ==================
        # ============================================================
        # === CLIENT_CHECK ===
        if self.state == RobotState.CLIENT_CHECK:
            self.set_face("SEARCHING_EYES")
            self.send_cmd("LEFT")
            if self.human_detected: 
                self.state = RobotState.CLIENT_ALIGN

        # === CLIENT_ALIGN ===
        elif self.state == RobotState.CLIENT_ALIGN:
            if not self.human_detected: 
                self.state = RobotState.CLIENT_CHECK
            else:
                self.set_face("SHOW_CLIENT_FEED")
                if abs(self.human_x_offset) < 0.2:
                    self.send_cmd("STOP")
                    self.state = RobotState.CLIENT_MOVE
                else:
                    if self.human_x_offset < 0:
                        self.send_cmd("LEFT")
                    else:
                        self.send_cmd("RIGHT")
                    
        # === CLIENT_MOVE ===
        elif self.state == RobotState.CLIENT_MOVE:
            if not self.human_detected: 
                self.state = RobotState.CLIENT_CHECK
            else:
                self.set_face("SHOW_FEED")
                if self.current_distance <= 50.0 or face_width > 180:
                    self.send_cmd("STOP")
                    self.state = RobotState.CLIENT_OPTION_SELECT
                else:
                    self.send_cmd("FORWARD")

        # === CLIENT_OPTION_SELECT ===
        elif self.state == RobotState.CLIENT_OPTION_SELECT:
            self.set_face("SHOW_UI_BUTTONS")
            self.send_cmd("STOP")
            if self.ui_confirmed:
                self.ui_confirmed = False
                self.state = RobotState.TABLE_CHECK

        # === TABLE_CHECK ===
        elif self.state == RobotState.TABLE_CHECK:
            self.set_face("SEARCHING_EYES")
            self.send_cmd("RIGHT")
            if self.table_detected: 
                self.state = RobotState.TABLE_ALIGN

        # === TABLE_ALIGN ===
        elif self.state == RobotState.TABLE_ALIGN:
            if not self.table_detected: 
                self.state = RobotState.TABLE_CHECK
            else:
                self.set_face("SHOW_FEED")
                if abs(self.table_x_offset) < 0.2:
                    self.send_cmd("STOP")
                    self.get_logger().info("Table Found and Aligned!")
                    self.state = RobotState.TABLE_MOVE
                else:
                    if self.table_x_offset < 0:
                        self.send_cmd("LEFT")
                    else:
                        self.send_cmd("RIGHT")

        # === TABLE_MOVE ===
        elif self.state == RobotState.TABLE_MOVE:
        # STOP CONDITIONS:
            # 1. Ultrasonic distance is close (< 40cm)
            # 2. OR the marker is large in the frame (e.g., > 180 pixels wide)
            if self.current_distance <= 40.0 or self.table_pixel_width > 180:
                self.send_cmd("STOP")
                self.state = RobotState.TABLE_OPTION_SELECT
            elif not self.table_detected:
                # If we lose the marker while moving, go back to checking
                self.state = RobotState.TABLE_CHECK
            else:
                self.send_cmd("FORWARD")
                
                    
        # === TABLE_OPTION_SELECT ===
        elif self.state == RobotState.TABLE_OPTION_SELECT:
            self.set_face("SHOW_UI_BUTTONS")
            self.send_cmd("STOP")
            if self.ui_confirmed:
                self.ui_confirmed = False
                self.state = RobotState.HOME_CHECK

        # === HOME_CHECK ===
        elif self.state == RobotState.HOME_CHECK:
            self.set_face("SEARCHING_EYES")
            self.send_cmd("LEFT")
            if self.home_detected: 
                self.state = RobotState.HOME_ALIGN
        
        # === HOME_ALIGN ===
        elif self.state == RobotState.HOME_ALIGN:
            if not self.home_detected: 
                self.state = RobotState.HOME_CHECK
            else:
                self.set_face("SHOW_FEED")
                if abs(self.home_x_offset) < 0.2:
                    self.send_cmd("STOP")
                    self.get_logger().info("Home Found and Aligned!")
                    self.state = RobotState.HOME_MOVE
                else:
                    if self.home_x_offset < 0:
                        self.send_cmd("LEFT")
                    else:
                        self.send_cmd("RIGHT")
        
        # === HOME_MOVE ===
        elif self.state == RobotState.HOME_MOVE:
        # STOP CONDITIONS:
            # 1. Ultrasonic distance is close (< 10cm)
            # 2. OR the marker is large in the frame (e.g., > 180 pixels wide)
            if self.current_distance <= 10.0 or self.home_pixel_width > 180:
                self.send_cmd("STOP")
                self.state = RobotState.CLIENT_CHECK
            elif not self.home_detected:
                # If we lose the marker while moving, go back to checking
                self.state = RobotState.HOME_CHECK
            else:
                self.send_cmd("FORWARD")
        
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
