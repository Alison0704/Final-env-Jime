import rclpy
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import String, Bool, Float32
from functools import partial

# --- Configuration ---
ARUCO_IDS = {
    'TABLE': 42,
    'HOME': 100
}

class RobotState(Enum):
    CUSTOMER_CHECK = 1
    ALIGN_HUMAN = 2
    MOVE_TO_HUMAN = 3
    GESTURE_AT_HOME = 4
    TABLE_CHECK = 5
    ALIGN_TABLE = 6
    MOVE_TO_TABLE = 7
    GESTURE_AT_TABLE = 8
    HOME_CHECK = 9
    ALIGN_HOME = 10
    MOVE_TO_HOME = 11
    OBSTACLE_AVOIDANCE = 12
    EMERGENCY_STOP = 13

class HostRobotLogic(Node):
    def __init__(self):
        super().__init__('host_robot_logic_node')
        
        # 1. Core State Tracking
        self.state = RobotState.CUSTOMER_CHECK
        self.previous_state = RobotState.CUSTOMER_CHECK
        
        # 2. Publishers
        self.speech_pub = self.create_publisher(String, '/robot_speech', 10)
        self.state_pub = self.create_publisher(String, '/robot_state_status', 10)
        self.esp32_command_pub = self.create_publisher(String, '/esp32/motor_commands', 10)
        self.face_pub = self.create_publisher(String, '/robot_face', 10) # For your HTML View

        # 3. Subscriptions
        self.create_subscription(Bool, '/customer_detected', partial(self.unified_cb, attr='trigger_customer'), 10)
        self.create_subscription(Bool, '/customer_reached', partial(self.unified_cb, attr='trigger_reached'), 10)
        self.create_subscription(Bool, '/gesture_detected', partial(self.unified_cb, attr='trigger_gesture'), 10)
        self.create_subscription(Bool, '/off_center', partial(self.unified_cb, attr='trigger_off_center'), 10)
        self.create_subscription(Float32, '/ultrasonic_distance', self.distance_cb, 10)
        self.create_subscription(Float32, '/camera/target_offset', self.offset_cb, 10)
        self.create_subscription(Bool, '/emergency_stop', self.emergency_cb, 10)
        self.create_subscription(String, '/aruco_id', self.aruco_cb, 10)
        self.create_subscription(String, '/ui_interaction', self.ui_callback, 10)        

        # 4. Flags & Data
        self.trigger_ui_confirm = False # Replaces trigger_gesture
        self.trigger_customer = False
        self.trigger_reached = False
        self.trigger_gesture = False
        self.trigger_off_center = False
        self.current_distance = 100.0  
        self.target_x_offset = 0.0     
        self.last_aruco_id = -1
        self.is_emergency = False

        # 5. FSM Timer (Adjusted to 10Hz for smoother control)
        self.timer = self.create_timer(0.1, self.state_machine_tick)
        
        self.get_logger().info("Jim-E Brain Online. Awaiting Customer...")
        self.update_face("NEUTRAL")

    # --- LED Helper ---
        def update_leds(self, cmd):
            """Turn off all LEDs, then turn on the one matching the current command."""
            for pin in self.led_pins:
                GPIO.output(pin, GPIO.LOW)
                
            if cmd == "FORWARD":
                GPIO.output(LED_FORWARD, GPIO.HIGH)
            elif cmd == "LEFT":
                GPIO.output(LED_LEFT, GPIO.HIGH)
            elif cmd == "RIGHT" or cmd == "ROTATE":
                GPIO.output(LED_RIGHT, GPIO.HIGH)
            elif cmd == "STOP":
                GPIO.output(LED_STOP, GPIO.HIGH)

    # --- Callbacks ---
    def ui_callback(self, msg):
        if msg.data == "CONFIRM_START":
            self.trigger_ui_confirm = True
            self.get_logger().info("UI Confirmation Received: Starting Table Lead.")
        elif msg.data == "EMERGENCY_STOP":
            self.is_emergency = True
    
    def aruco_cb(self, msg):
        try:
            self.last_aruco_id = int(msg.data)
        except ValueError:
            pass
            
    def distance_cb(self, msg): self.current_distance = msg.data
    def offset_cb(self, msg): self.target_x_offset = msg.data
    def emergency_cb(self, msg): self.is_emergency = msg.data
    def unified_cb(self, msg, attr): setattr(self, attr, msg.data)

    def send_esp32_command(self, cmd_string):
        self.esp32_command_pub.publish(String(data=cmd_string))

    def update_face(self, expression):
        # Sends signal to your HTML/CSS eyes
        self.face_pub.publish(String(data=expression))

    def change_state(self, new_state):
        if self.state == new_state: return  
        
        # Only save previous state if it's a 'working' state
        if self.state != RobotState.OBSTACLE_AVOIDANCE:
            self.previous_state = self.state
            
        self.state = new_state
        self.state_pub.publish(String(data=self.state.name))
        self.get_logger().info(f"TRANSITION: {self.previous_state.name} -> {self.state.name}")
        self.on_state_entry()

    def on_state_entry(self):
        """Triggers once per state change"""
        if self.state == RobotState.GESTURE_AT_HOME:
            self.update_face("HAPPY")
            self.speech_pub.publish(String(data="Hello! Would you like to find a table? Give me a thumbs up!"))
        elif self.state == RobotState.OBSTACLE_AVOIDANCE:
            self.update_face("SURPRISED")
        elif self.state == RobotState.CUSTOMER_CHECK:
            self.update_face("SEARCHING")

    def state_machine_tick(self):
        # --- 1. PRIORITY SAFETY ---
        if self.is_emergency or self.current_distance < 20.0:
            self.send_esp32_command("STOP")
            if self.state != RobotState.OBSTACLE_AVOIDANCE:
                self.change_state(RobotState.OBSTACLE_AVOIDANCE)
            return

        # --- 2. FSM LOGIC ---
        
        # OBSTACLE RECOVERY
        if self.state == RobotState.OBSTACLE_AVOIDANCE:
            if self.current_distance > 35.0: # Hysteresis buffer
                self.change_state(self.previous_state)
            else:
                self.send_esp32_command("STOP")

        # SEARCHING PHASE
        elif self.state in [RobotState.CUSTOMER_CHECK, RobotState.TABLE_CHECK, RobotState.HOME_CHECK]:
            self.handle_search_logic()

        # ALIGNMENT PHASE
        elif self.state in [RobotState.ALIGN_HUMAN, RobotState.ALIGN_TABLE, RobotState.ALIGN_HOME]:
            self.handle_alignment_logic()

        # NAVIGATION PHASE
        elif self.state in [RobotState.MOVE_TO_HUMAN, RobotState.MOVE_TO_TABLE, RobotState.MOVE_TO_HOME]:
            self.handle_navigation_logic()

        # GESTURE/INTERACTION PHASE
        elif self.state in [RobotState.GESTURE_AT_HOME, RobotState.GESTURE_AT_TABLE]:
            self.handle_gesture_logic()
        

    # --- Logic Helpers ---

    def handle_search_logic(self):
        self.send_esp32_command("ROTATE")
        if self.state == RobotState.CUSTOMER_CHECK and self.trigger_customer:
            self.change_state(RobotState.ALIGN_HUMAN)
        elif self.state == RobotState.TABLE_CHECK and self.last_aruco_id == ARUCO_IDS['TABLE']:
            self.change_state(RobotState.ALIGN_TABLE)
        elif self.state == RobotState.HOME_CHECK and self.last_aruco_id == ARUCO_IDS['HOME']:
            self.change_state(RobotState.ALIGN_HOME)

    def handle_alignment_logic(self):
        if abs(self.target_x_offset) < 0.15: # Center threshold
            self.send_esp32_command("STOP")
            if self.state == RobotState.ALIGN_HUMAN: self.change_state(RobotState.MOVE_TO_HUMAN)
            elif self.state == RobotState.ALIGN_TABLE: self.change_state(RobotState.MOVE_TO_TABLE)
            elif self.state == RobotState.ALIGN_HOME: self.change_state(RobotState.MOVE_TO_HOME)
        else:
            cmd = "LEFT" if self.target_x_offset < 0 else "RIGHT"
            self.send_esp32_command(cmd)

    def handle_navigation_logic(self):
        if self.trigger_reached:
            self.send_esp32_command("STOP")
            self.trigger_reached = False # Reset flag
            if self.state == RobotState.MOVE_TO_HUMAN: self.change_state(RobotState.GESTURE_AT_HOME)
            elif self.state == RobotState.MOVE_TO_TABLE: self.change_state(RobotState.GESTURE_AT_TABLE)
            elif self.state == RobotState.MOVE_TO_HOME: self.change_state(RobotState.CUSTOMER_CHECK)
        else:
            self.send_esp32_command("FORWARD")

    def handle_gesture_logic(self):
        self.send_esp32_command("STOP")
            # Now checking for UI button press instead of a physical gesture
            if self.trigger_ui_confirm:
                self.trigger_ui_confirm = False 
                if self.state == RobotState.GESTURE_AT_HOME: 
                    self.change_state(RobotState.TABLE_CHECK)
                elif self.state == RobotState.GESTURE_AT_TABLE: 
                    self.change_state(RobotState.HOME_CHECK)
                    
def main(args=None):
    rclpy.init(args=args)
    node = HostRobotLogic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

