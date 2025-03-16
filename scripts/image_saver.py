#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw', 
            self.image_callback,
            10)
        
        self.bridge = CvBridge()
        self.image_count = 0
        self.save_path = os.path.join(os.getcwd(), "images")

        # Create the directory if it doesn't exist
        os.makedirs(self.save_path, exist_ok=True)
        
        self.get_logger().info("Image saver node started. Waiting for images...")

    def image_callback(self, msg):
        """Callback function to save images from the subscribed topic."""
        if self.image_count < 30:
            try:
                # Convert ROS Image message to OpenCV format
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

                # Save the image
                image_filename = os.path.join(self.save_path, f"image_{self.image_count:02d}.jpg")
                cv2.imwrite(image_filename, cv_image)

                self.get_logger().info(f"Saved image {self.image_count + 1}/30: {image_filename}")
                
                self.image_count += 1

                # # Once 10 images are saved, stop the node
                # if self.image_count >= 30:
                #     self.get_logger().info("Successfully saved 10 images. Shutting down...")
                #     rclpy.shutdown()

            except Exception as e:
                self.get_logger().error(f"Failed to process image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
