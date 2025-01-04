import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class Distance(Node):
    def __init__(self):
        super().__init__('distance_node')

        # Local variable to cache the distance
        self.distance = 0.0

        # Subscriber to the /distance_traveled topic
        self.subscription = self.create_subscription(
            Float32,
            '/distance_traveled',
            self.distance_callback,
            10
        )

        # Log message to indicate the node has started
        self.get_logger().info('Distance Cache Node has started.')

    def distance_callback(self, msg):
        """Callback function for the /distance_traveled topic."""
        self.distance = msg.data
        #self.get_logger().info(f'Distance updated: {self.cached_distance:.5f}')


def main(args=None):
    rclpy.init(args=args)
    node = Distance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanly shut down the node
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()