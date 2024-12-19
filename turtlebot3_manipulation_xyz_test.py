#!/usr/bin/env python 
# Set linear and angular values of Turtlesim's speed and turning. 

import os
import select
import sys
import getkey

import rclpy	# Needed to create a ROS node 
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist    # Message that moves base
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from std_msgs.msg import Header
from rclpy.node import Node
from rclpy.qos import QoSProfile
import math


j1_z_offset = 77
r1 = 130
r2 = 124
r3 = 150


th1_offset = - math.atan2(0.024, 0.128)
th2_offset = - 0.5*math.pi - th1_offset

# author : karl.kwon (mrthinks@gmail.com)
# r1 : distance J0 to J1
# r2 : distance J1 to J2
# r3 : distance J0 to J2
def solv2(r1, r2, r3):
  d1 = (r3**2 - r2**2 + r1**2) / (2*r3)
  d2 = (r3**2 + r2**2 - r1**2) / (2*r3)

  s1 = math.acos(d1 / r1)
  s2 = math.acos(d2 / r2)

  return s1, s2

# author : karl.kwon (mrthinks@gmail.com)
# x, y, z : relational position from J0 (joint 0)
# r1 : distance J0 to J1
# r2 : distance J1 to J2
# r3 : distance J2 to J3
# sr1 : angle between z-axis to J0->J1
# sr2 : angle between J0->J1 to J1->J2
# sr3 : angle between J1->J2 to J2->J3 (maybe always parallel)
def solv_robot_arm2(x, y, z, r1, r2, r3):
  z = z + r3 - j1_z_offset

  Rt = math.sqrt(x**2 + y**2 + z**2)
  Rxy = math.sqrt(x**2 + y**2)
  St = math.asin(z / Rt)
#   Sxy = math.acos(x / Rxy)
  Sxy = math.atan2(y, x)

  s1, s2 = solv2(r1, r2, Rt)

  sr1 = math.pi/2 - (s1 + St)
  sr2 = s1 + s2
  sr2_ = sr1 + sr2
  sr3 = math.pi - sr2_

  return Sxy, sr1, sr2, sr3, St, Rt


usage = """
Control Your OpenManipulator!
---------------------------
Joint Space Control:
- Joint1 : Increase (Y), Decrease (H)
- Joint2 : Increase (U), Decrease (J)
- Joint3 : Increase (I), Decrease (K)
- Joint4 : Increase (O), Decrease (L)

w: forward
s: backward
d: turn right
a: turn left

<space>: stop

INIT : (1)

CTRL-C to quit
"""

joint_angle_delta = 0.1  # radian


class Turtlebot3ManipulationTest(Node): 
	# settings = None
	# if os.name != 'nt':
	# 	settings = termios.tcgetattr(sys.stdin)
	qos = QoSProfile(depth=10)
        
	def __init__(self): 

		super().__init__('turtlebot3_manipulation_test')
		key_value = ''
		
		self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
		self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
		self.gripper_action_client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')
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

		Sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(100, 0, 0, r1, r2, r3)


		current_time = self.get_clock().now()
		self.trajectory_msg.header = Header()
#		self.trajectory_msg.header.stamp = current_time.to_msg()
		self.trajectory_msg.header.frame_id = ''
		self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

		point = JointTrajectoryPoint()
		point.positions = [Sxy, sr1 + th1_offset, sr2 + th2_offset, sr3]
		point.velocities = [0.0] * 4
		point.accelerations = [0.0] * 4
		point.time_from_start.sec = 0
		point.time_from_start.nanosec = 500

		self.trajectory_msg.points = [point]

		self.joint_pub.publish(self.trajectory_msg)

	def send_gripper_goal(self, position):
		goal = GripperCommand.Goal()
		goal.command.position = position
		goal.command.max_effort = -1.0
    
		if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
			self.get_logger().error("Gripper action server not available!")
			return
          
		self.gripper_action_client.send_goal_async(goal)
        

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

            elif key_value == 'w':
                node.move_cmd.linear.x = 0.1
                node.move_cmd.angular.z = 0.
                node.cmd_vel.publish(node.move_cmd) 
                print('forward')
            elif key_value == 's':
                node.move_cmd.linear.x = -0.1
                node.move_cmd.angular.z = 0.
                node.cmd_vel.publish(node.move_cmd) 
                print('backward')
            elif key_value == 'd':
                node.move_cmd.linear.x = 0.
                node.move_cmd.angular.z = -0.1
                node.cmd_vel.publish(node.move_cmd) 
                print('right')
            elif key_value == 'a':
                node.move_cmd.linear.x = 0.
                node.move_cmd.angular.z = 0.1
                node.cmd_vel.publish(node.move_cmd) 
                print('left')
            elif key_value == ' ':
                node.move_cmd.linear.x = 0.
                node.move_cmd.angular.z = 0.
                node.cmd_vel.publish(node.move_cmd) 
                print('stop')

            elif key_value == '=':
                node.send_gripper_goal(-0.015)
                print('open')
            elif key_value == '-':
                node.send_gripper_goal(0.01)
                print('close')

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
