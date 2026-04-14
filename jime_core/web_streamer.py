import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage  # Using Compressed for better performance
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from flask import Flask, Response
import threading
import numpy as np
from ultralytics import YOLO

# Initialize Flask
app = Flask(__name__)
latest_frame = None
bridge = CvBridge()
current_state = "CLIENT_CHECK"

# Load YOLO model (Nano version for Pi performance)
model = YOLO('/home/ubuntu/ros2_ws/yolov8n.pt')

class WebStreamer(Node):
    def __init__(self):
        super().__init__('web_streamer')
        
        # Subscribe to the COMPRESSED topic found in your 'topic list'
        self.subscription = self.create_subscription(
            CompressedImage, 
            '/camera/image_raw/compressed', 
            self.image_callback, 
            10)
            
        # 2. Subscribe to Jim-E's state to trigger the UI overlays
        self.state_sub = self.create_subscription(
            String,
            '/robot_state_status',
            self.state_callback,
            10)
        
        self.get_logger().info("Jim-E Web Streamer (Compressed) Started")

    def state_callback(self, msg):
        global current_state
        current_state = msg.data

    def image_callback(self, msg):
        global latest_frame
        try:
            # 3. Decode the Compressed ROS Image to OpenCV
            # This is much faster than handling raw image messages
            frame = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            
            # Resize for consistent UI (matching the camera node output)
            frame = cv2.resize(frame, (640, 360))
            h, w, _ = frame.shape
            cx, cy = w // 2, h // 2

            # --- CUSTOMER_CHECK UI OVERLAY ---
            if current_state == "CUSTOMER_CHECK":
                # A. Draw Scanning Grid
                grid_color = (0, 100, 0)
                cv2.line(frame, (w//3, 0), (w//3, h), grid_color, 1)
                cv2.line(frame, (2*w//3, 0), (2*w//3, h), grid_color, 1)
                cv2.line(frame, (0, h//3), (w, h//3), grid_color, 1)
                cv2.line(frame, (0, 2*h//3), (w, 2*h//3), grid_color, 1)
                
                # B. Draw Center Crosshair
                cv2.line(frame, (cx-20, cy), (cx+20, cy), (0, 255, 0), 2)
                cv2.line(frame, (cx, cy-20), (cx, cy+20), (0, 255, 0), 2)

                # C. YOLO Detection (Filter for "person" - Class 0)
                # Lower conf to 0.3 for better detection in varying light
                results = model.predict(frame, classes=[0], conf=0.3, verbose=False)
                
                for r in results:
                    if len(r.boxes) > 0:
                        # Log to terminal for debugging
                        print(f"TARGET DETECTED: {len(r.boxes)} subject(s)")
                        
                        for box in r.boxes:
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            
                            # Draw SOLID box over the face/head area (Top 30%)
                            head_h = int((y2 - y1) * 0.3)
                            cv2.rectangle(frame, (x1, y1), (x2, y1 + head_h), (0, 255, 0), -1)
                            
                            # Add "TARGET LOCKED" label
                            cv2.putText(frame, "TARGET LOCKED", (x1, y1 - 10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # D. Status Overlay Text
                cv2.putText(frame, "SCANNING...", (20, 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Update the global frame for Flask
            latest_frame = frame

        except Exception as e:
            print(f"Streamer Error: {e}")

def gen_frames():
    global latest_frame
    while True:
        if latest_frame is not None:
            # Encode frame as JPEG for the web stream
            ret, buffer = cv2.imencode('.jpg', latest_frame)
            if not ret:
                continue
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def main(args=None):
    rclpy.init(args=args)
    node = WebStreamer()

    # Start ROS in background thread, Flask in main thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
