import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription_rgb = self.create_subscription(
            CompressedImage,
            'rgb_image/compressed_image',
            self.listener_callback_rgb,
            10)
        self.subscription_rgb  # prevent unused variable warning

    def listener_callback_rgb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode to color image
        cv2.imshow('RGB Image', image_np)
        cv2.waitKey(1)  # Display the image until a keypress

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
