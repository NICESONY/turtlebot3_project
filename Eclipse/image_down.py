#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32 # Import Int32 for downsample factor control
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')

        self.bridge = CvBridge()
        self._downsample_factor = 2 # Initial default value for downsample factor
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Create subscription for dynamic downsample factor control
        self.downsample_factor_sub = self.create_subscription(
            Int32,
            '/image_processor/downsample_factor',
            self.downsample_factor_callback,
            10 # QoS history depth
        )

        # Create publisher for the processed image topic
        self.image_pub = self.create_publisher(Image, '/follow_img', 10)
        self.i = 0;

    def downsample_factor_callback(self, msg: Int32):
        """
        Callback for receiving dynamic downsample factor updates.
        """
        new_factor = msg.data
        if new_factor > 0:
            if new_factor != self._downsample_factor:
                self.get_logger().info(f'📝 Updating downsample factor from {self._downsample_factor} to {new_factor}.')
                self._downsample_factor = new_factor
        else:
            self.get_logger().warn(f'🚫 Received invalid downsample factor: {new_factor}. Must be a positive integer. Keeping current factor: {self._downsample_factor}')


    def image_callback(self, msg):
        """
        Callback function for the image subscription.
        Processes the incoming raw image: converts to grayscale, downsamples, and publishes.
        """
        try:
            # Convert ROS Image message to OpenCV image (BGR format for color processing)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to convert image from ROS to OpenCV: {e}')
            return

        # 1. Convert to Grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # self.get_logger().debug('✅ Image converted to grayscale.') # Too verbose, uncomment if needed for deep debugging

        # 2. Downsample Resolution
        # Use the currently active downsample_factor
        current_downsample_factor = self._downsample_factor
        original_height, original_width = gray_image.shape
        new_width = int(original_width / current_downsample_factor)
        new_height = int(original_height / current_downsample_factor)

        # Ensure new dimensions are at least 1x1 to avoid errors for very large factors
        if new_width < 1: new_width = 1
        if new_height < 1: new_height = 1

        downsampled_image = cv2.resize(gray_image, (new_width, new_height),
                                        interpolation=cv2.INTER_AREA)
        # self.get_logger().debug(f'✅ Image downsampled from {original_width}x{original_height} '
        #                         f'to {new_width}x{new_height} with factor {current_downsample_factor}.')


        try:
            self.i += 1
            # Convert the processed OpenCV image (grayscale) back to a ROS Image message
            processed_msg = self.bridge.cv2_to_imgmsg(downsampled_image, encoding='mono8')
            processed_msg.header = msg.header
            if i % 6 == 0:
                self.image_pub.publish(processed_msg)
            # self.get_logger().debug('✅ Processed image published.') # Too verbose, uncomment if needed for deep debugging
        except Exception as e:
            self.get_logger().error(f'❌ Failed to convert and publish processed image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()