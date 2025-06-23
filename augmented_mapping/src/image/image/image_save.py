import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time


class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/camera',  # Replace with your camera topic
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.save_path = 'images'  # Update this to your desired path
        os.makedirs(self.save_path, exist_ok=True)
        self.counter = 70
        self.image_data = None


        # Create a timer that calls save_image every 5 seconds
        self.timer = self.create_timer(5.0, self.save_image)  # Timer set to 5 seconds


    def listener_callback(self, data):
        # Store the latest image received
        self.image_data = data


    def save_image(self):
        if self.image_data:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(self.image_data, "bgr8")
            # Save image
            image_name = os.path.join(self.save_path, f"image_{self.counter:04}.png")
            cv2.imwrite(image_name, cv_image)
            self.get_logger().info(f"Saved {image_name}")
            self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    image_saver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
