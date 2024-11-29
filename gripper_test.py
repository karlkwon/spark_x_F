#!/usr/bin/env python 
# Set linear and angular values of Turtlesim's speed and turning. 

import rclpy	# Needed to create a ROS node 
# from geometry_msgs.msg import Twist    # Message that moves base
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from rclpy.node import Node
import math

class Turtlebot3ManipulationTest(Node): 
	def __init__(self): 

		super().__init__('ControlTurtlesim')
		
		self.gripper_action_client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')
		self.timer = self.create_timer(1.0, self.timer_callback)

		self.gripper_state = 0

	def timer_callback(self):
		print('.')

		if self.gripper_state == 0:
			self.send_gripper_goal(0.025)  # Open
			self.gripper_state = 1
		else:
			self.send_gripper_goal(-0.015)  # Close
			self.gripper_state = 0

	def send_gripper_goal(self, position):
		# """Gripper goal 설정"""
		goal = GripperCommand.Goal()
		goal.command.position = position
		goal.command.max_effort = -1.0

		if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
			self.get_logger().error("Gripper action server not available!")
			return

		self.gripper_action_client.send_goal_async(goal)

def main(args=None):
    rclpy.init(args=args)
    node = Turtlebot3ManipulationTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__== "__main__": 
	main() 
