import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  
from osr_interfaces.msg import Status

class CoreStatus(Node):
    def __init__(self):
        super().__init__('status_subscriber')

        # Initialize subscriber
        self.subscription = self.create_subscription(
            Status, 
            '/motor_controllers/status',
            self.status_callback,
            10  # QoS depth
        )

        # Subscribe to CPU temperature
        self.cpu_temp_subscription = self.create_subscription(
            Float32,
            '/cpu_temperature',
            self.cpu_temperature_callback,
            10  # QoS depth
        )

        # Initialize the core_status dictionary
        self.core_status = {
            'voltage': None,  # Default value
            'cpu_temperature': None   # CPU temperature
        }

        self.get_logger().info("CoreStatus node has been started.")
        
    def status_callback(self, msg):
        try:
            # Extract 'battery' voltage from the message
            # Assuming `battery` is a field in the Status message
            self.core_status['voltage'] = msg.battery  # Update with actual field name

            # Log the cached voltage for debugging
            #self.get_logger().info(f"Updated voltage: {self.core_status['voltage']} V")
        except AttributeError:
            self.get_logger().error("Field 'battery' does not exist in the message.")

    def cpu_temperature_callback(self, msg):
        try:
            # Extract CPU temperature from the message
            self.core_status['cpu_temperature'] = msg.data

            # Log the updated CPU temperature for debugging
            # self.get_logger().info(f"Updated CPU Temperature: {self.core_status['cpu_temperature']:.2f} °C")
        except AttributeError:
            self.get_logger().error("Failed to process CPU temperature message.")

def main(args=None):
    rclpy.init(args=args)
    node = CoreStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()