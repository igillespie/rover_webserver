import rclpy # type: ignore
from web_publishers.joystick_publisher import JoystickPublisher
from robot_sensors.drive_state import DriveStateSubscriber
from robot_sensors.corner_state import CornerStateSubscriber
from robot_sensors.main_camera import MainCameraSubscriber
from robot_sensors.battery import BatteryStateSubscriber
from robot_sensors.core_status import CoreStatus
from robot_sensors.distance import Distance

from rclpy.executors import MultiThreadedExecutor # type: ignore
import threading

class ROSController:
    def __init__(self):
        """Initialize the ROSController."""
        rclpy.init()
        self.executor = MultiThreadedExecutor()
        self.nodes = {}
        self.running = False

    def start_publisher_nodes(self):
        """Start all publisher nodes."""
        joystick_publisher = JoystickPublisher()
        self.nodes['joystick_publisher'] = joystick_publisher
        self.executor.add_node(joystick_publisher)

    def start_subscriber_nodes(self):
        """Start all subscriber nodes."""
        drive_state_subscriber = DriveStateSubscriber()
        self.nodes["drive_state_subscriber"] = drive_state_subscriber
        self.executor.add_node(drive_state_subscriber)

        corner_state_subscriber = CornerStateSubscriber()
        self.nodes["corner_state_subscriber"] = corner_state_subscriber
        self.executor.add_node(corner_state_subscriber)

        main_camera_subscriber = MainCameraSubscriber()
        self.nodes["main_camera_subscriber"] = main_camera_subscriber
        self.executor.add_node(main_camera_subscriber)

        battery_state_subscriber = BatteryStateSubscriber()
        self.nodes["battery_state_subscriber"] = battery_state_subscriber
        self.executor.add_node(battery_state_subscriber)

        core_status = CoreStatus()
        self.nodes["core_status"] = core_status
        self.executor.add_node(core_status)

        distance = Distance()
        self.nodes["distance"] = distance
        self.executor.add_node(distance)
       
        

    def spin_all_nodes(self):
        """Spin all managed nodes in a separate thread."""
        self.running = True
        thread = threading.Thread(target=self._spin_executor)
        thread.start()
        return thread

    def _spin_executor(self):
        """Spin the MultiThreadedExecutor."""
        try:
            print("Spinning all nodes using MultiThreadedExecutor")
            self.executor.spin()
        except Exception as e:
            print(f"Error during spinning: {e}")
        finally:
            self.shutdown()

    def get_all_nodes(self):
        return self.nodes
        
    def get_node(self, node_name):
        return self.nodes.get(node_name)

    def shutdown(self):
        """Shut down all nodes and stop spinning."""
        if rclpy.ok():  # Check if the context is still active
            self.running = False
            self.executor.shutdown()
            rclpy.shutdown()
        else:
            print("ROS context already shut down.")