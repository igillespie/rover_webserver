import rclpy
from rclpy.node import Node
from turn_in_place_interfaces.msg import TurnCommand


class TurnInPlacePublisher(Node):
    def __init__(self):
        super().__init__('turn_in_place_publisher')
        self.publisher_ = self.create_publisher(TurnCommand, 'turn_command', 10)
        self.get_logger().info("TurnInPlacePublisher is ready. You can send commands.")

    def publish_command(self, angle_degrees, angular_speed_deg_per_sec):
        msg = TurnCommand()
        msg.angle_degrees = float(angle_degrees)
        msg.angular_speed_deg_per_sec = float(angular_speed_deg_per_sec)
        self.publisher_.publish(msg)
        # self.get_logger().info(
        #     f'Published TurnCommand: angle_degrees={angle_degrees}, angular_speed_deg_per_sec={angular_speed_deg_per_sec}'
        # )