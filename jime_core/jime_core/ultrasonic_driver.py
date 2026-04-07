import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time

# Use RPi.GPIO or gpiozero
import RPi.GPIO as GPIO

class UltrasonicSensorNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')
        
        # --- GPIO Setup ---
        self.TRIG = 23  # Update to your physical TRIG pin
        self.ECHO = 24  # Update to your physical ECHO pin
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)
        
        # --- ROS2 Setup ---
        self.publisher_ = self.create_publisher(Float32, '/ultrasonic_distance', 10)
        
        # Timer to read sensor at 10Hz (matching your main logic)
        self.timer = self.create_timer(0.1, self.publish_distance)
        self.get_logger().info("Ultrasonic Sensor Node Started.")

    def get_distance(self):
        """Reads the HC-SR04 sensor and returns distance in cm."""
        try:
            # Clean Trigger
            GPIO.output(self.TRIG, False)
            time.sleep(0.01) # Short settle time
            
            # Trigger pulse
            GPIO.output(self.TRIG, True)
            time.sleep(0.00001)
            GPIO.output(self.TRIG, False)

            # 1. Wait for Echo to go HIGH (with 0.05s timeout)
            timeout = time.time() + 0.05
            while GPIO.input(self.ECHO) == 0:
                start_time = time.time()
                if start_time > timeout:
                    return 100.0 # Timeout

            # 2. Wait for Echo to go LOW (with 0.05s timeout)
            timeout = time.time() + 0.05
            while GPIO.input(self.ECHO) == 1:
                stop_time = time.time()
                if stop_time > timeout:
                    return 100.0 # Timeout

            duration = stop_time - start_time
            distance = duration * 17150
            
            # Sanity check for sensor limits
            if distance > 400 or distance < 2:
                return 100.0
                
            return round(distance, 2)
            
        except Exception as e:
            self.get_logger().error(f"Sensor Read Error: {e}")
            return 100.0 # Return safe default

    def publish_distance(self):
        dist = self.get_distance()
        msg = Float32()
        msg.data = float(dist)
        self.publisher_.publish(msg)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
