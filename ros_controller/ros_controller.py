import rclpy
from web_publishers.joystick_publisher import JoystickPublisher
from robot_sensors.drive_state import DriveStateSubscriber

class ROSController:
    def __init__(self):
        """Initialize the ROSController."""
        rclpy.init()
        self.nodes = {}
        self.running = False

    def start_publisher_nodes(self):
        """Start all publisher nodes."""
        # Initialize and store the JoystickPublisher
        joystick_publisher = JoystickPublisher()
        self.nodes['joystick_publisher'] = joystick_publisher

    def start_subscriber_nodes(self):
        """Start all subscriber nodes."""
        drive_state_subscriber = DriveStateSubscriber()
        self.nodes["drive_state_subscriber"] = drive_state_subscriber
        # Add additional subscribers here, if needed
        pass

    def get_node(self, node_name):
        """Retrieve a managed node by name."""
        return self.nodes.get(node_name)

    def spin_nodes(self):
        """Spin all managed nodes."""
        self.running = True
        try:
            for node in self.nodes.values():
                rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self):
        """Cleanly shutdown all nodes and ROS2."""
        if self.running:
            for node in self.nodes.values():
                node.destroy_node()
            rclpy.shutdown()
            self.running = False