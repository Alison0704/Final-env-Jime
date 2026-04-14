import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class StateMonitor(Node):
    def __init__(self):
        super().__init__('state_monitor')

        # Create a QoS profile that perfectly matches the 'jime_brain' publisher
        # This ensures the 'Subscription count' becomes 1.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.subscription = self.create_subscription(
            String,
            '/robot_status',
            self.listener_callback,
            qos_profile) # Corrected: passing the object, not an integer

        self.last_state = None
        self.get_logger().info('--- JIM-E State Monitor Active ---')
        self.get_logger().info('Listening for transitions on /robot_status...')

    def listener_callback(self, msg):
        current_data = msg.data
        
        # Only log if the message has changed to keep the terminal clean
        if current_data != self.last_state:
            # Visual formatting for easy SSH monitoring
            output = (
                f"\n" + "="*40 + 
                f"\n [TRANSITION DETECTED]" +
                f"\n DATA: {current_data}" +
                f"\n" + "="*40
            )
            self.get_logger().info(output)
            self.last_state = current_data

def main(args=None):
    rclpy.init(args=args)
    node = StateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('State Monitor shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
