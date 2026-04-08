import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Switched to String
import serial

class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        
        self.declare_parameter('port', '/dev/ttyUSB0')
        port = self.get_parameter('port').get_parameter_value().string_value

        try:
            self.ser = serial.Serial(port, 115200, timeout=0.1)
            self.get_logger().info(f"Connected to ESP32 on {port}")
        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")
            exit()

        # Listening for simple words on the 'robot_commands' topic
        self.subscription = self.create_subscription(
            String,
            'robot_cmd',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        command = msg.data.upper() # Ensure it's uppercase
        valid_commands = ["FORWARD", "LEFT", "RIGHT", "STOP"]
        
        if command in valid_commands:
            self.get_logger().info(f"Passing to ESP32: {command}")
            self.ser.write(f"{command}\n".encode('utf-8'))
        else:
            self.get_logger().warn(f"Invalid command ignored: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = ESP32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.ser.write("STOP\n".encode('utf-8'))
    finally:
        node.destroy_node()
        rclpy.shutdown()
