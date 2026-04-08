import rclpy
from rclpy.node import Node
import subprocess
import os
import signal

class MicroRosBridgeNode(Node):
    def __init__(self):
        super().__init__('microros_bridge') # This name must match what you look for
        self.get_logger().info('Starting micro-ROS Docker Bridge...')
        
        # Start Docker
        # Update this line in your start_bridge.py
        self.docker_cmd = [
            "docker", "run", "--rm",
            "--name", "jime_bridge_agent",
            "--privileged",
            "-v", "/dev:/dev",
            "microros/micro-ros-agent:humble",
            "serial", "--dev", "/dev/ttyUSB0", "-b", "115200", "-v6" # Added -b 115200
        ]

        try:
            self.get_logger().info("Starting micro-ROS Serial Bridge...")
            # Use the variable name we just defined
            self.proc = subprocess.Popen(self.docker_cmd) 
            self.get_logger().info("Docker Agent process spawned on /dev/ttyUSB0.")
        except Exception as e:
            self.get_logger().error(f"Failed to start Docker: {e}")
            
        # Create a timer that does nothing just to keep the node active in the graph
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Just a heartbeat to keep the node visible
        pass

    def stop_bridge(self):
        self.get_logger().info('Stopping Docker Agent...')
        os.system("docker stop jime_bridge_agent > /dev/null 2>&1")
        if hasattr(self, 'process'):
            self.process.terminate()

def main(args=None):
    rclpy.init(args=args)
    node = MicroRosBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_bridge()
        node.destroy_node()
        rclpy.shutdown()
