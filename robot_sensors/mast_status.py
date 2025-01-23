import rclpy
from rclpy.node import Node
from mast_control_interfaces.msg import PanTiltCommand, PanTiltState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class MastStatus(Node):
    def __init__(self):
        super().__init__('mast_status')

        # Cached state
        self.current_state = None

        # Publisher for pan_tilt_command
        self.command_pub = self.create_publisher(PanTiltCommand, '/pan_tilt_command', 10)

        # Subscriber for pan_tilt_state
        self.state_sub = self.create_subscription(
            PanTiltState,
            '/pan_tilt_state',
            self.state_callback,
            10
        )

        self.get_logger().info("MastStatus node has been started.")

    def state_callback(self, msg):
        """Callback to cache the most recent pan_tilt_state."""
        self.current_state = msg
        self.get_logger().debug(f"Cached state: pan={msg.pan_position}, tilt={msg.tilt_position}, status={msg.status}")


    def set_mast_pan_tilt_angles_incremental(self, pan_delta, tilt_delta):
        if self.current_state is None:
            self.get_logger().warning("No current state available. Cannot set incremental angles.")
        else:
            new_pan = self.current_state.pan_position + pan_delta
            new_tilt = self.current_state.tilt_position + tilt_delta
            self.set_mast_pan_angle(new_pan, new_tilt)

    def set_mast_pan_angle(self, pan_angle, tilt_angle):
        """Set the mast pan angle and publish the command."""
        if self.current_state is None:
            self.get_logger().warning("No current state available. Cannot validate angles.")
        # else:
        #     self.get_logger().info(f"Current state: pan={self.current_state.pan_position}, tilt={self.current_state.tilt_position}, status={self.current_state.status}")

        # Validate angles (example range: 0-300 degrees)
        if not (0 <= pan_angle <= 300):
            raise ValueError(f"Pan angle {pan_angle} out of range (0-300)")
        if not (0 <= tilt_angle <= 300):
            raise ValueError(f"Tilt angle {tilt_angle} out of range (0-300)")

        # Create and publish the command
        command = PanTiltCommand()
        command.pan_angle = pan_angle
        command.tilt_angle = tilt_angle
        # command.speed = speed
        self.command_pub.publish(command)

        #self.get_logger().info(f"Published command: pan={pan_angle}, tilt={tilt_angle}")

def main(args=None):
    rclpy.init(args=args)
    node = MastStatus()

    try:
        # Example usage: set the mast pan angle
        node.set_mast_pan_angle(90.0, 45.0)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()