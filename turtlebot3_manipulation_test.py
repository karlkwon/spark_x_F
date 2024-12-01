#!/usr/bin/env python 
# Set linear and angular values of Turtlesim's speed and turning. 

import os
import select
import sys
import getkey

import rclpy	# Needed to create a ROS node 
from geometry_msgs.msg import Twist    # Message that moves base
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from rclpy.node import Node
import math


usage = """
Control Your OpenManipulator!
---------------------------
Joint Space Control:
- Joint1 : Increase (Y), Decrease (H)
- Joint2 : Increase (U), Decrease (J)
- Joint3 : Increase (I), Decrease (K)
- Joint4 : Increase (O), Decrease (L)

INIT : (1)

CTRL-C to quit
"""

joint_angle_delta = 0.05  # radian


class Turtlebot3ManipulationTest(Node): 
	# settings = None
	# if os.name != 'nt':
	# 	settings = termios.tcgetattr(sys.stdin)
        
	def __init__(self): 

		super().__init__('turtlebot3_manipulation_test')
		key_value = ''
		
		self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
		self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
		# self.timer = self.create_timer(1.0, self.timer_callback)

		# Twist is geometry_msgs for linear and angular velocity 
		self.move_cmd = Twist() 
		# Linear speed in x in meters/second is + (forward) or 
		#    - (backwards) 
		self.move_cmd.linear.x = 1.3   # Modify this value to change speed 
		# Turn at 0 radians/s 
		self.move_cmd.angular.z = 0.8 
		# Modify this value to cause rotation rad/s 

		self.trajectory_msg = JointTrajectory()

		current_time = self.get_clock().now()
		self.trajectory_msg.header = Header()
		self.trajectory_msg.header.stamp = current_time.to_msg()
		self.trajectory_msg.header.frame_id = ''
		self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

		point = JointTrajectoryPoint()
		# point.positions = [0.003, math.pi / 4.0, -0.489, 2.041]
		point.positions = [0.0] * 4
		point.velocities = [0.0] * 4
		point.time_from_start.sec = 3
		point.time_from_start.nanosec = 0

		self.trajectory_msg.points = [point]

		# self.joint_pub.publish(self.trajectory_msg)

	# def timer_callback(self):
	# 	print('.')
	# 	self.joint_pub.publish(self.trajectory_msg)
	# 	self.cmd_vel.publish(self.move_cmd) 


node = None

def main(args=None):
    # rclpy.init(args=args)

    try:
        rclpy.init()
    except Exception as e:
        print(e)

    try:
        node = Turtlebot3ManipulationTest()
    except Exception as e:
        print(e)

    try:
        while(rclpy.ok()):
            # rclpy.spin_once(node)
            key_value = getkey.getkey()
            
            if key_value == '1':
                node.trajectory_msg.points[0].positions = [0.0] * 4
                node.joint_pub.publish(node.trajectory_msg)
                print('joint1 +')
            elif key_value == 'y':
                node.trajectory_msg.points[0].positions[0] += joint_angle_delta
                node.joint_pub.publish(node.trajectory_msg)
                print('joint1 +')
            elif key_value == 'h':
                node.trajectory_msg.points[0].positions[0] -= joint_angle_delta
                node.joint_pub.publish(node.trajectory_msg)
                print('joint1 -')
            elif key_value == 'u':
                node.trajectory_msg.points[0].positions[1] += joint_angle_delta
                node.joint_pub.publish(node.trajectory_msg)
                print('joint2 +')
            elif key_value == 'j':
                node.trajectory_msg.points[0].positions[1] -= joint_angle_delta
                node.joint_pub.publish(node.trajectory_msg)
                print('joint2 -')
            elif key_value == 'i':
                node.trajectory_msg.points[0].positions[2] += joint_angle_delta
                node.joint_pub.publish(node.trajectory_msg)
                print('joint3 +')
            elif key_value == 'k':
                node.trajectory_msg.points[0].positions[2] -= joint_angle_delta
                node.joint_pub.publish(node.trajectory_msg)
                print('joint3 -')
            elif key_value == 'o':
                node.trajectory_msg.points[0].positions[3] += joint_angle_delta
                node.joint_pub.publish(node.trajectory_msg)
                print('joint4 +')
            elif key_value == 'l':
                node.trajectory_msg.points[0].positions[3] -= joint_angle_delta
                node.joint_pub.publish(node.trajectory_msg)
                print('joint4 -')
            elif key_value == 'q':
                break
            

    except Exception as e:
        print(e)

    finally:
        # if os.name != 'nt':
        #     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__== "__main__": 
	main() 
