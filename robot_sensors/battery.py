import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState  # Import the BatteryState message

class BatteryStateSubscriber(Node):
    def __init__(self):
        super().__init__('battery_state_subscriber')
        # Initialize the subscriber
        self.subscription = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_state_callback,
            10  # QoS (queue size)
        )
        self.battery_state = {}  # Dictionary to cache voltage and current
        self.get_logger().info('BatteryStateSubscriber has been started.')

    def battery_state_callback(self, msg):
        # Update the cache with voltage and current values
        self.battery_state['voltage'] = msg.voltage
        self.battery_state['current'] = msg.current

        # Log the cached values (for debugging)
        #self.get_logger().info(f"Cached battery data: {self.battery_cache}")


def main(args=None):
    rclpy.init(args=args)
    node = BatteryStateSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()