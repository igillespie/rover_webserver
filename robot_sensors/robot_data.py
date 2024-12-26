import rclpy  # type: ignore
from .drive_state import DriveState, DriveStateSubscriber

class RobotSensors:
    """
    A class to manage all robot sensor subscriptions.
    """
    def __init__(self):
        print("Beginning Init")
        self.drive_state = DriveState()
        self.drive_state_subscriber = DriveStateSubscriber(self.drive_state)

    def spin(self):
        """
        Starts spinning the ROS 2 node to process subscription callbacks.
        """
        try:
            print("RobotSensors is running...")
            rclpy.spin(self.drive_state_subscriber)  # Spin the node to process callbacks
        except KeyboardInterrupt:
            print("Shutting down RobotSensors...")
        finally:
            self.shutdown()

    def shutdown(self):
        """
        Shuts down the ROS 2 nodes and the ROS client library.
        """
        self.drive_state_subscriber.destroy_node()
        rclpy.shutdown()