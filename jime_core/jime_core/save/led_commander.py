import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class LedCommander(Node):
    def __init__(self):
        super().__init__('led_commander')
        self.publisher_ = self.create_publisher(String, 'robot_cmd', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.commands = ["FORWARD", "LEFT", "RIGHT", "ROTATE_SLOW", "STOP"]
        self.get_logger().info('Jim-E LED Commander started. Sending random commands...')

    def timer_callback(self):
        msg = String()
        msg.data = random.choice(self.commands)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = LedCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
