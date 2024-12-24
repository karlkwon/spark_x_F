import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QImage,QPixmap
from PyQt5 import uic
import cv2,imutils

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np


form_class = uic.loadUiType("simple_qt_ros.ui")[0]


class WindowClass(QMainWindow, form_class):
	def __init__(self):
		super().__init__()
		self.setupUi(self)

		self.btn_1.clicked.connect(self.button1Function)
		self.btn_2.clicked.connect(self.button2Function)
		# self.label_1.setText('HAHA')

		self.isClosed = False

	def button1Function(self):
		print("btn_1 clicked")

	def button2Function(self):
		print("btn_2 clicked")
		# image = cv2.imread('SavedImage.jpg')
		# image = self.cvimage_to_label(image)
		# self.label_1.setPixmap(QPixmap.fromImage(image))

	def showImage(self, image_np):
		image = self.cvimage_to_label(image_np)
		self.label_1.setPixmap(QPixmap.fromImage(image))

	def cvimage_to_label(self,image):
		image = imutils.resize(image,width = 640)
		image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
		image = QImage(image,
			image.shape[1],
			image.shape[0],
			QImage.Format_RGB888)

		return image

	def closeEvent(self, event):
		print("closeEvent")
		self.isClosed = True


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.image_np = None
        self.subscription_rgb = self.create_subscription(
            CompressedImage,
            'rgb_image/compressed_image',
            self.listener_callback_rgb,
            10)
        self.subscription_rgb  # prevent unused variable warning

    def listener_callback_rgb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode to color image
        # cv2.imshow('RGB Image', image_np)
        # cv2.waitKey(1)  # Display the image until a keypress


if __name__ == "__main__":
	app = QApplication(sys.argv)
	myWindow = WindowClass()

	rclpy.init(args=None)
	image_subscriber = ImageSubscriber()

	myWindow.show()

	while(myWindow.isClosed == False):
		rclpy.spin_once(image_subscriber, timeout_sec=0)
		if(image_subscriber.image_np is not None):
			myWindow.showImage(image_subscriber.image_np)
		app.processEvents()
		# print(a)

	image_subscriber.destroy_node()
	rclpy.shutdown()
	# app.exec_()