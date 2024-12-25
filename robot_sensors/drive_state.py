import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from sensor_msgs.msg import JointState  # type: ignore


class DriveState:
    """
    A class to subscribe to the /drive_state topic and store its data for later use.
    This class avoids being tightly coupled with the ROS 2 system.
    """
    def __init__(self):
        self._latest_data = None

    def get_latest_data(self):
        """Returns the latest data received from the /drive_state topic."""
        return self._latest_data

    def set_latest_data(self, data):
        """Sets the latest data, used by the ROS 2 subscriber."""
        self._latest_data = data

    def get_velocity(self):
        try:
            return self._latest_data["velocities"]
        except:
            return None


class DriveStateSubscriber(Node):
    """
    A ROS 2 Node that subscribes to the /drive_state topic and updates the DriveState object.
    """
    def __init__(self, drive_state: DriveState):
        super().__init__('drive_state_subscriber')
        self.drive_state = drive_state
        self.subscription = self.create_subscription(
            JointState,  # Adjust to the correct m/essage type for /drive_state
            '/drive_state',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent the subscription from being garbage collected

    def listener_callback(self, msg):
        """Callback function for the subscriber."""
        # Store the full message or specific fields as needed
        self.drive_state.set_latest_data({
            "names": msg.name,
            "positions": msg.position,
            "velocities": msg.velocity,
            "efforts": msg.effort
        })
        # Log or print relevant details for debugging
        # self.get_logger().info(f"Received JointState with {len(msg.name)} joints")
        # print(f"Velocities: {msg.velocity}")