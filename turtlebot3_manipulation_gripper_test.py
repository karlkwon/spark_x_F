#!/usr/bin/env python 
# Set linear and angular values of Turtlesim's speed and turning. 

import rclpy	# Needed to create a ROS node 
from geometry_msgs.msg import Twist    # Message that moves base
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from rclpy.node import Node
import math

class Turtlebot3ManipulationTest(Node): 
	def __init__(self): 

		super().__init__('ControlTurtlesim')
		
		self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
		self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
		self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
		# self.gripper_action_client = ActionClient(self, JointTrajectory, '/gripper_controller/joint_trajectory')
		self.timer = self.create_timer(1.0, self.timer_callback)

		# Twist is geometry_msgs for linear and angular velocity 
		self.move_cmd = Twist() 
		# Linear speed in x in meters/second is + (forward) or 
		#    - (backwards) 
		self.move_cmd.linear.x = 1.3   # Modify this value to change speed 
		# Turn at 0 radians/s 
		self.move_cmd.angular.z = 0.8 
		# Modify this value to cause rotation rad/s 

		trajectory_msg = JointTrajectory()

		current_time = self.get_clock().now()
		trajectory_msg.header = Header()
		trajectory_msg.header.stamp = current_time.to_msg()
		trajectory_msg.header.frame_id = ''
		trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

		point = JointTrajectoryPoint()
		point.positions = [0.003, math.pi / 4.0, -0.489, 2.041]
		point.velocities = [0.0] * 4
		point.time_from_start.sec = 3
		point.time_from_start.nanosec = 0

		trajectory_msg.points = [point]

		self.joint_pub.publish(trajectory_msg)
		self.gripper_state = 0

	def timer_callback(self):
		print('.')
		self.cmd_vel.publish(self.move_cmd)

		if self.gripper_state == 0:
			self.send_gripper_goal(0.025)  # Open
			self.gripper_state = 1
		else:
			self.send_gripper_goal(-0.015)  # Close
			self.gripper_state = 0

	def send_gripper_goal(self, position):
		trajectory_msg = JointTrajectory()

		current_time = self.get_clock().now()
		trajectory_msg.header = Header()
		trajectory_msg.header.stamp = current_time.to_msg()
		trajectory_msg.header.frame_id = ''
		trajectory_msg.joint_names = ['gripper_left_joint', ]

		point = JointTrajectoryPoint()
		point.positions = [position, ]
		point.velocities = [0.0]
		point.time_from_start.sec = 3
		point.time_from_start.nanosec = 0

		trajectory_msg.points = [point]

		self.gripper_pub.publish(trajectory_msg)

		# """Gripper goal 설정"""
		# goal = GripperCommand.Goal()
		# goal.command.position = position
		# goal.command.max_effort = -1.0

		# if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
		# 	self.get_logger().error("Gripper action server not available!")
		# 	return

		# self.gripper_action_client.send_goal_async(goal)

def main(args=None):
    rclpy.init(args=args)
    node = Turtlebot3ManipulationTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__== "__main__": 
	main() 
