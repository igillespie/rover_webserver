import rclpy # type: ignore
from rclpy.node import Node # type: ignore
#from geometry_msgs.msg import Twist# type: ignore
from sensor_msgs.msg import Joy

# ROS2 Node setup
class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('web_joystick_publisher')
        self.publisher = self.create_publisher(Joy, 'joy', 10)

    def publish_joystick_data(self, x, y):
        """
        Publish joystick data as sensor_msgs/Joy.
        """
        # Default axes and buttons (PlayStation 5 has 8 axes and 14 buttons)
        axes = [0.0] * 8
        buttons = [0] * 14

        # Map JavaScript joystick data to appropriate axes
        axes[3] = float(x)  # Axis 3: Angular velocity (Yaw)
        axes[4] = float(y)  # Axis 4: Linear velocity (Forward/Backward)

        # Enable movement by setting button 7
        buttons[7] = 1  # Enable button

        # Construct the Joy message
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()  # Add current timestamp
        msg.header.frame_id = "web_joystick"  # Set frame_id
        msg.axes = axes
        msg.buttons = buttons

        # Publish the message
        self.publisher.publish(msg)
        #self.get_logger().info(f"Published /joy: axes={msg.axes}, buttons={msg.buttons}")
