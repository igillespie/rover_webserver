import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from sensor_msgs.msg import CompressedImage  # type: ignore

class MainCameraSubscriber(Node):
    """
    A ROS 2 Node that subscribes to the /camera_frames/compressed topic and caches the latest compressed JPEG image.
    """

    def __init__(self):
        super().__init__('main_camera_subscriber')

        # Initialize cache for the latest image
        self.latest_image = None

        # Create a subscription to the /camera_frames/compressed topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera_frames/compressed',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent the subscription from being garbage collected

        self.get_logger().info("MainCameraSubscriber initialized and listening on /camera_frames/compressed.")

    def listener_callback(self, msg):
        """Callback function for the subscriber."""
        #print(f"Received image at timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        try:
            # Cache the received image message
            self.latest_image = msg
            self.get_logger().debug("Received and cached a new compressed image.")
        except Exception as e:
            self.get_logger().error(f"Error in listener_callback: {e}")

    def get_cached_image(self):
        """Retrieve the cached compressed image."""
        if self.latest_image:
            return self.latest_image
        else:
            self.get_logger().warn("No image cached yet.",  throttle_duration_sec=5)
            return None

if __name__ == "__main__":
    rclpy.init()
    main_camera_subscriber = MainCameraSubscriber()
    try:
        rclpy.spin(main_camera_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        main_camera_subscriber.destroy_node()
        rclpy.shutdown()