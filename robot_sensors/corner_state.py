import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from sensor_msgs.msg import JointState  # type: ignore

class CornerStateSubscriber(Node):
    """
    A ROS 2 Node that subscribes to the /drive_state topic and updates the DriveState object.
    """
    def __init__(self):
        super().__init__('corner_state_subscriber')
        # self.drive_state = DriveState()
        self.corner_state = []
        self.subscription = self.create_subscription(
            JointState,  # Adjust to the correct m/essage type for /drive_state
            '/corner_state',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent the subscription from being garbage collected

    def listener_callback(self, msg):
        """Callback function for the subscriber."""
        # Combine the data into a list of dictionaries for better usability
        # print(f"Message: {msg}")
        data = [
            {
                "name": name,
                "position": position,
            }
            for name, position, _, _ in zip(msg.name, msg.position, msg.velocity, msg.effort)
        ]

        # Cache the structured data
        self.corner_state = data
        # print(self.corner_state)
        # print(f"Data: {self.corner_state}")
        # for d, da in enumerate(data):
        #     print(f"Position of servo {d}: {da['position']}")
        # print(f"Corner State Data: {self.corner_state}")

if __name__ == "__main__":
    rclpy.init()
    corner_state_subscriber = CornerStateSubscriber()
    rclpy.spin(corner_state_subscriber)
    corner_state_subscriber.destroy_node()
    rclpy.shutdown()