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
        
    def get_fronts(self):
        """
        Lazily compute and return the left-front and right-front wheel data.

        Returns:
            dict: A dictionary containing left-front and right-front wheel data.
        """
        left_front = next(
            (wheel for wheel in self._latest_data if wheel["name"] == "drive_left_front"), None
        )
        right_front = next(
            (wheel for wheel in self._latest_data if wheel["name"] == "drive_right_front"), None
        )

        return {
            "left_front": left_front,
            "right_front": right_front,
        }


class DriveStateSubscriber(Node):
    """
    A ROS 2 Node that subscribes to the /drive_state topic and updates the DriveState object.
    """
    def __init__(self):
        super().__init__('drive_state_subscriber')
        self.drive_state = DriveState()
        self.subscription = self.create_subscription(
            JointState,  # Adjust to the correct m/essage type for /drive_state
            '/drive_state',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent the subscription from being garbage collected

    def listener_callback(self, msg):
        """Callback function for the subscriber."""
        # Combine the data into a list of dictionaries for better usability
        data = [
            {
                "name": name,
                "position": position,
                "velocity": velocity,
                "effort": effort
            }
            for name, position, velocity, effort in zip(msg.name, msg.position, msg.velocity, msg.effort)
        ]

        # Cache the structured data
        self.drive_state.set_latest_data(data)
        # Log or print relevant details for debugging
        # self.get_logger().info(f"Received JointState with {len(msg.name)} joints")
        # print(f"Velocities: {msg.velocity}")