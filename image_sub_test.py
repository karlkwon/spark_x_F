import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()
        self.subscription_rgb = self.create_subscription(
            Image,
            'rgb_image',
            self.listener_callback_rgb,
            10)
        self.subscription_rgb  # prevent unused variable warning

    def listener_callback_rgb(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow('RGB Image', cv_image)
        cv2.waitKey(1)  # Display the image until a keypress

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
