import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QImage,QPixmap
from PyQt5 import uic
import cv2,imutils

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np

import queue
import time
from threading import Thread


form_class = uic.loadUiType("simple_qt_ros.ui")[0]


class WindowClass(QMainWindow, form_class):
	def __init__(self, operation_queue):
		super().__init__()
		self.setupUi(self)

		self.btn_1.clicked.connect(self.button1Function)
		self.btn_2.clicked.connect(self.button2Function)
		# self.label_1.setText('HAHA')

		self.isClosed = False
		self.operation_queue = operation_queue

	def button1Function(self):
		print("btn_1 clicked")

	def button2Function(self):
		print("btn_2 clicked")
		# image = cv2.imread('SavedImage.jpg')
		# image = self.cvimage_to_label(image)
		# self.label_1.setPixmap(QPixmap.fromImage(image))

	def editAddFunction(self, text):
		self.edit_1.append(text)

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
		self.operation_queue.put(' ')
		self.operation_queue.put(' ')

		print("closeEvent")

		time.sleep(0.1)

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


def ros_thread(operation_queue, event_queue, sleep_time):
	time_p = time.time()

	while(True):
		try:
			d = operation_queue.get_nowait()
			if d is not None:
				break
		except queue.Empty:
			pass

		time_c = time.time()
		if (time_c - time_p) > (sleep_time):
			event_queue.put('sleep_time: ', sleep_time)
			time_p = time_c

		time.sleep(0.01)
		# time.sleep(sleep_time)

	print('exit : ', sleep_time)


if __name__ == "__main__":
	app = QApplication(sys.argv)
	operation_queue = queue.Queue()
	myWindow = WindowClass(operation_queue)

	rclpy.init(args=None)
	image_subscriber = ImageSubscriber()

	myWindow.show()

	event_queue1 = queue.Queue()
	Thread(target = ros_thread, args=(operation_queue, event_queue1, 1), daemon=True).start()
	event_queue2 = queue.Queue()
	Thread(target = ros_thread, args=(operation_queue, event_queue2, 2), daemon=True).start()


	while(myWindow.isClosed == False):

		try:
			d1 = event_queue1.get_nowait()
			if d1 is not None:
				myWindow.editAddFunction('1' + d1)
		except queue.Empty:
			pass

		try:
			d2 = event_queue2.get_nowait()
			if d2 is not None:
				myWindow.editAddFunction('2' + d2)
		except queue.Empty:
			pass

		rclpy.spin_once(image_subscriber, timeout_sec=0)
		if(image_subscriber.image_np is not None):
			myWindow.showImage(image_subscriber.image_np)
		app.processEvents()
		# print(a)

	image_subscriber.destroy_node()
	rclpy.shutdown()
	# app.exec_()
