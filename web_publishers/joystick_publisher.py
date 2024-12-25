import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from std_msgs.msg import String # type: ignore


# ROS2 Node setup
class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher')
        self.publisher = self.create_publisher(String, 'joystick_data', 10)

    def publish_joystick_data(self, x, y):
        msg = String()
        msg.data = f"x: {x}, y: {y}"
        self.publisher.publish(msg)
        self.get_logger().info(f"Published joystick data: {msg.data}")
