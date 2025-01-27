# import rclpy
# from rclpy.node import Node
# from turn_in_place_interfaces.msg import TurnCommand
# from std_msgs.msg import Float32

# class TurnInPlacePublisher(Node):
#     def __init__(self):
#         super().__init__('turn_in_place_publisher')
#         self.publisher_ = self.create_publisher(TurnCommand, 'turn_command', 10)
#         self.get_logger().info("TurnInPlacePublisher is ready. You can send commands.")

#     def publish_command(self, angle_degrees, angular_speed_deg_per_sec):
#         msg = TurnCommand()
#         msg.angle_degrees = float(angle_degrees)
#         msg.angular_speed_deg_per_sec = float(angular_speed_deg_per_sec)
#         self.publisher_.publish(msg)
#         # self.get_logger().info(
#         #     f'Published TurnCommand: angle_degrees={angle_degrees}, angular_speed_deg_per_sec={angular_speed_deg_per_sec}'
#         # )

# class MoveDistancePublisher(Node):
#     def __init__(self):
#         super().__init__('move_distance_publisher')
#         self.publisher_ = self.create_publisher(Float32, 'move_distance', 10)
#         self.get_logger().info("MoveDistancePublisher is ready. You can send commands.")

#     def publish_command(self, meters):
#         msg = Float32()
#         msg.data = float(meters)  # Ensure meters is cast to float
#         self.publisher_.publish(msg)
#         self.get_logger().info(f'Published MoveDistance: {meters} meters')

import rclpy
from rclpy.node import Node
from turn_in_place_interfaces.msg import TurnCommand
from std_msgs.msg import Float32


class MovementCommandPublisher(Node):
    def __init__(self):
        super().__init__('movement_command_publisher')

        # Publishers for turn and move commands
        self.turn_publisher = self.create_publisher(TurnCommand, 'turn_command', 10)
        self.move_distance_publisher = self.create_publisher(Float32, 'move_distance', 10)

        self.get_logger().info("MovementCommandPublisher is ready.")

    def publish_turn_in_place_command(self, angle_degrees, angular_speed_deg_per_sec):
        """
        Publishes a TurnCommand message to the 'turn_command' topic.
        :param angle_degrees: The angle in degrees to turn (positive for right, negative for left).
        :param angular_speed_deg_per_sec: The angular speed in degrees per second.
        """
        msg = TurnCommand()
        msg.angle_degrees = float(angle_degrees)
        #msg.angular_speed_deg_per_sec = float(angular_speed_deg_per_sec)
        self.turn_publisher.publish(msg)
        # self.get_logger().info(
        #     f"Published TurnCommand: angle_degrees={angle_degrees}, angular_speed_deg_per_sec={angular_speed_deg_per_sec}"
        # )

    def publish_move_distance_command(self, meters):
        """
        Publishes a Float32 message to the 'move_distance' topic.
        :param meters: The distance in meters to move (positive for forward, negative for backward).
        """
        msg = Float32()
        msg.data = float(meters)
        self.move_distance_publisher.publish(msg)
        # self.get_logger().info(f"Published MoveDistance: {meters} meters")