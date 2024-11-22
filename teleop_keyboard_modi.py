#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Will Son

from concurrent.futures import ThreadPoolExecutor
from math import exp
import os
import select
import sys
import rclpy

from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
# from rclpy.executors import Executor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from threading import Timer

import math
import numpy as np

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

present_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
present_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

debug = True
task_position_delta = 0.01  # meter
joint_angle_delta = 0.05  # radian
path_time = 0.5  # second

r1 = 130
r2 = 124
r3 = 126


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

  J0 = (0, 0, 0)
  J1 = (J0[0] + r1 * math.sin(sr1)  * math.cos(Sxy),
        J0[1] + r1 * math.sin(sr1)  * math.sin(Sxy),
        J0[2] + r1 * math.cos(sr1))
  J2 = (J1[0] + r2 * math.sin(sr1 + sr2) * math.cos(Sxy),
        J1[1] + r2 * math.sin(sr1 + sr2) * math.sin(Sxy),
        J1[2] + r2 * math.cos(sr1 + sr2))
  J3 = (J2[0] + r3 * math.sin(sr1 + sr2 + sr3) * math.cos(Sxy),
        J2[1] + r3 * math.sin(sr1 + sr2 + sr3) * math.sin(Sxy),
        J2[2] + r3 * math.cos(sr1 + sr2 + sr3))

  return J0, J1, J2, J3, Sxy, sr1, sr2, sr3, St, Rt


usage = """
Control Your OpenManipulator modified!
---------------------------
Task Space Control:
         (Forward, X+)
              W                   Q (Upward, Z+)
(Left, Y+) A     D (Right, Y-)    Z (Downward, Z-)
              X 
        (Backward, X-)

Joint Space Control:
- Joint1 : Increase (Y), Decrease (H)
- Joint2 : Increase (U), Decrease (J)
- Joint3 : Increase (I), Decrease (K)
- Joint4 : Increase (O), Decrease (L)
- Gripper: Open     (G),    Close (F) | Fully Open (V), Fully Close (B)

INIT : (1)
HOME : (2)

CTRL-C to quit
"""

e = """
Communications Failed
"""


class TeleopKeyboardModi(Node):

    qos = QoSProfile(depth=10)
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    def __init__(self):
        super().__init__('teleop_keyboard_modi')
        key_value = ''

        # Create joint_states subscriber
        self.joint_state_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            self.qos)
        self.joint_state_subscription

        # Create kinematics_pose subscriber
        self.kinematics_pose_subscription = self.create_subscription(
            KinematicsPose,
            'kinematics_pose',
            self.kinematics_pose_callback,
            self.qos)
        self.kinematics_pose_subscription

        # Create manipulator state subscriber
        self.open_manipulator_state_subscription = self.create_subscription(
            OpenManipulatorState,
            'states',
            self.open_manipulator_state_callback,
            self.qos)
        self.open_manipulator_state_subscription

        # Create Service Clients
        self.goal_joint_space = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.goal_task_space = self.create_client(SetKinematicsPose, 'goal_task_space_path')
        self.tool_control = self.create_client(SetJointPosition, 'goal_tool_control')
        self.goal_joint_space_req = SetJointPosition.Request()
        self.goal_task_space_req = SetKinematicsPose.Request()
        self.tool_control_req = SetJointPosition.Request()

    def send_goal_task_space(self):
        self.goal_task_space_req.end_effector_name = 'gripper'
        self.goal_task_space_req.kinematics_pose.pose.position.x = goal_kinematics_pose[0]
        self.goal_task_space_req.kinematics_pose.pose.position.y = goal_kinematics_pose[1]
        self.goal_task_space_req.kinematics_pose.pose.position.z = goal_kinematics_pose[2]
        self.goal_task_space_req.kinematics_pose.pose.orientation.w = goal_kinematics_pose[3]
        self.goal_task_space_req.kinematics_pose.pose.orientation.x = goal_kinematics_pose[4]
        self.goal_task_space_req.kinematics_pose.pose.orientation.y = goal_kinematics_pose[5]
        self.goal_task_space_req.kinematics_pose.pose.orientation.z = goal_kinematics_pose[6]
        self.goal_task_space_req.path_time = path_time

        try:
            self.goal_task_space.call_async(self.goal_task_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Kinematic Pose failed %r' % (e,))

    def send_goal_joint_space(self):
        self.goal_joint_space_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        self.goal_joint_space_req.joint_position.position = [
            goal_joint_angle[0],
            goal_joint_angle[1] + th1_offset,
            goal_joint_angle[2] + th2_offset,
            goal_joint_angle[3],
            goal_joint_angle[4]
        ]
        self.goal_joint_space_req.path_time = path_time

        try:
            self.goal_joint_space.call_async(self.goal_joint_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))

    def send_tool_control_request(self):
        self.tool_control_req.joint_position.joint_name = ['gripper']
        self.tool_control_req.joint_position.position = [goal_joint_angle[4]]
        self.tool_control_req.path_time = path_time

        try:
            future = self.tool_control.call_async(self.tool_control_req)
            future.add_done_callback(self.tool_control_response)
        except Exception as e:
            self.get_logger().error(f"Service call to 'goal_tool_control' failed: {e}")

    def tool_control_response(self, future):
        try:
            result = future.result()
            self.get_logger().info(f'Tool control succeeded: {result}')
        except Exception as e:
            self.get_logger().error(f'Tool control failed: {e}')


    def kinematics_pose_callback(self, msg):
        present_kinematics_pose[0] = msg.pose.position.x
        present_kinematics_pose[1] = msg.pose.position.y
        present_kinematics_pose[2] = msg.pose.position.z
        present_kinematics_pose[3] = msg.pose.orientation.w
        present_kinematics_pose[4] = msg.pose.orientation.x
        present_kinematics_pose[5] = msg.pose.orientation.y
        present_kinematics_pose[6] = msg.pose.orientation.z

    def joint_state_callback(self, msg):
        # Ensure that there are at least 5 joint positions in the message
        if len(msg.position) >= 5:
            present_joint_angle[0] = msg.position[0]
            present_joint_angle[1] = msg.position[1] - th1_offset
            present_joint_angle[2] = msg.position[2] - th2_offset
            present_joint_angle[3] = msg.position[3]
            present_joint_angle[4] = msg.position[4]
        else:
            self.get_logger().warn('Received JointState message with insufficient joint positions.')

    def open_manipulator_state_callback(self, msg):
        if msg.open_manipulator_moving_state == 'STOPPED':
            for index in range(0, 7):
                goal_kinematics_pose[index] = present_kinematics_pose[index]
            for index in range(0, 5):
                goal_joint_angle[index] = present_joint_angle[index]


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print_present_values()
    return key


def print_present_values():
    print('Joint Angle(Rad): [{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}]'.format(
        present_joint_angle[0],
        present_joint_angle[1],
        present_joint_angle[2],
        present_joint_angle[3],
        present_joint_angle[4]))
    print('Kinematics Pose(Pose X, Y, Z | Orientation W, X, Y, Z): {:.3f}, {:.3f}, {:.3f} | {:.3f}, {:.3f}, {:.3f}, {:.3f}'.format(
        present_kinematics_pose[0],
        present_kinematics_pose[1],
        present_kinematics_pose[2],
        present_kinematics_pose[3],
        present_kinematics_pose[4],
        present_kinematics_pose[5],
        present_kinematics_pose[6]))


def main():
    settings = None
    tutorial_x = 100
    tutorial_y = 0
    tutorial_z = 0

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    try:
        rclpy.init()
    except Exception as e:
        print(e)

    try:
        teleop_keyboard = TeleopKeyboardModi()
    except Exception as e:
        print(e)

    try:
        while(rclpy.ok()):
            rclpy.spin_once(teleop_keyboard)

            print('tutorial_position: [{:.3f}, {:.3f}, {:.3f}] - {:.3f}'.format(tutorial_x, tutorial_y, tutorial_z, math.atan2(tutorial_y, tutorial_x)))

            key_value = get_key(settings)
            # if key_value == 'w':
            #     goal_kinematics_pose[0] = prev_goal_kinematics_pose[0] + task_position_delta
            #     teleop_keyboard.send_goal_task_space()
            # elif key_value == 'x':
            #     goal_kinematics_pose[0] = prev_goal_kinematics_pose[0] - task_position_delta
            #     teleop_keyboard.send_goal_task_space()
            # elif key_value == 'a':
            #     goal_kinematics_pose[1] = prev_goal_kinematics_pose[1] + task_position_delta
            #     teleop_keyboard.send_goal_task_space()
            # elif key_value == 'd':
            #     goal_kinematics_pose[1] = prev_goal_kinematics_pose[1] - task_position_delta
            #     teleop_keyboard.send_goal_task_space()

            # elif key_value == 'q':
            #     goal_kinematics_pose[2] = prev_goal_kinematics_pose[2] + task_position_delta
            #     teleop_keyboard.send_goal_task_space()
            # elif key_value == 'z':
            #     goal_kinematics_pose[2] = prev_goal_kinematics_pose[2] - task_position_delta
            #     teleop_keyboard.send_goal_task_space()
                
            # elif key_value == 'y':
            if key_value == 'y':
                goal_joint_angle[0] = prev_goal_joint_angle[0] + joint_angle_delta
                teleop_keyboard.send_goal_joint_space()
            elif key_value == 'h':
                goal_joint_angle[0] = prev_goal_joint_angle[0] - joint_angle_delta
                teleop_keyboard.send_goal_joint_space()
            elif key_value == 'u':
                goal_joint_angle[1] = prev_goal_joint_angle[1] + joint_angle_delta
                teleop_keyboard.send_goal_joint_space()
            elif key_value == 'j':
                goal_joint_angle[1] = prev_goal_joint_angle[1] - joint_angle_delta
                teleop_keyboard.send_goal_joint_space()
            elif key_value == 'i':
                goal_joint_angle[2] = prev_goal_joint_angle[2] + joint_angle_delta
                teleop_keyboard.send_goal_joint_space()
            elif key_value == 'k':
                goal_joint_angle[2] = prev_goal_joint_angle[2] - joint_angle_delta
                teleop_keyboard.send_goal_joint_space()
            elif key_value == 'o':
                goal_joint_angle[3] = prev_goal_joint_angle[3] + joint_angle_delta
                teleop_keyboard.send_goal_joint_space()
            elif key_value == 'l':
                goal_joint_angle[3] = prev_goal_joint_angle[3] - joint_angle_delta
                teleop_keyboard.send_goal_joint_space()
            elif key_value == 'f':  # Close gripper
                goal_joint_angle[4] = max(prev_goal_joint_angle[4] - 0.002, -0.01)  # Max close limit
                prev_goal_joint_angle[4] = goal_joint_angle[4]
                teleop_keyboard.send_tool_control_request()

            elif key_value == 'g':  # Open gripper
                goal_joint_angle[4] = min(prev_goal_joint_angle[4] + 0.002, 0.01)  # Max open limit
                prev_goal_joint_angle[4] = goal_joint_angle[4]
                teleop_keyboard.send_tool_control_request()

            elif key_value == 'v':  # Fully open
                goal_joint_angle[4] = 0.01
                prev_goal_joint_angle[4] = goal_joint_angle[4]
                teleop_keyboard.send_tool_control_request()

            elif key_value == 'b':  # Fully close
                goal_joint_angle[4] = -0.01
                prev_goal_joint_angle[4] = goal_joint_angle[4]
                teleop_keyboard.send_tool_control_request()

            elif key_value == 'x':
                tutorial_y -= 10
                
                J0, J1, J2, J3, sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(tutorial_x, tutorial_y, tutorial_z, r1, r2, r3)

                goal_joint_angle[0] = sxy
                goal_joint_angle[1] = sr1
                goal_joint_angle[2] = sr2
                goal_joint_angle[3] = sr3
                goal_joint_angle[4] = 0.0
                path_time = 5.0
                teleop_keyboard.send_goal_joint_space()

            elif key_value == 'w':
                tutorial_y += 10
                
                J0, J1, J2, J3, sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(tutorial_x, tutorial_y, tutorial_z, r1, r2, r3)

                goal_joint_angle[0] = sxy
                goal_joint_angle[1] = sr1
                goal_joint_angle[2] = sr2
                goal_joint_angle[3] = sr3
                goal_joint_angle[4] = 0.0
                path_time = 5.0
                teleop_keyboard.send_goal_joint_space()

            elif key_value == 'a':
                tutorial_x -= 10
                
                J0, J1, J2, J3, sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(tutorial_x, tutorial_y, tutorial_z, r1, r2, r3)

                goal_joint_angle[0] = sxy
                goal_joint_angle[1] = sr1
                goal_joint_angle[2] = sr2
                goal_joint_angle[3] = sr3
                goal_joint_angle[4] = 0.0
                path_time = 5.0
                teleop_keyboard.send_goal_joint_space()

            elif key_value == 'd':
                tutorial_x += 10
                
                J0, J1, J2, J3, sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(tutorial_x, tutorial_y, tutorial_z, r1, r2, r3)

                goal_joint_angle[0] = sxy
                goal_joint_angle[1] = sr1
                goal_joint_angle[2] = sr2
                goal_joint_angle[3] = sr3
                goal_joint_angle[4] = 0.0
                path_time = 5.0
                teleop_keyboard.send_goal_joint_space()

            elif key_value == 'q':
                tutorial_z += 10
                
                J0, J1, J2, J3, sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(tutorial_x, tutorial_y, tutorial_z, r1, r2, r3)

                goal_joint_angle[0] = sxy
                goal_joint_angle[1] = sr1
                goal_joint_angle[2] = sr2
                goal_joint_angle[3] = sr3
                goal_joint_angle[4] = 0.0
                path_time = 5.0
                teleop_keyboard.send_goal_joint_space()

            elif key_value == 'z':
                tutorial_z -= 10
                
                J0, J1, J2, J3, sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(tutorial_x, tutorial_y, tutorial_z, r1, r2, r3)

                goal_joint_angle[0] = sxy
                goal_joint_angle[1] = sr1
                goal_joint_angle[2] = sr2
                goal_joint_angle[3] = sr3
                goal_joint_angle[4] = 0.0
                path_time = 5.0
                teleop_keyboard.send_goal_joint_space()

            elif key_value == '1':
                J0, J1, J2, J3, sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(tutorial_x, tutorial_y, tutorial_z, r1, r2, r3)

                goal_joint_angle[0] = sxy
                goal_joint_angle[1] = sr1
                goal_joint_angle[2] = sr2
                goal_joint_angle[3] = sr3
                goal_joint_angle[4] = 0.0
                path_time = 5.0
                teleop_keyboard.send_goal_joint_space()
            elif key_value == '2':
                goal_joint_angle[4] = 0.01
                prev_goal_joint_angle[4] = goal_joint_angle[4]
                teleop_keyboard.send_tool_control_request()
            elif key_value == '3':
                goal_joint_angle[0] = 0.0
                goal_joint_angle[1] = -1.2
                goal_joint_angle[2] = 0.35
                goal_joint_angle[3] = 1.7
                goal_joint_angle[4] = 0.0
                path_time = 5.0
                teleop_keyboard.send_goal_joint_space()
            elif key_value == '4':
                goal_joint_angle[0] = 0.0
                goal_joint_angle[1] = 0.4
                goal_joint_angle[2] = 0.0
                goal_joint_angle[3] = 1.3
                goal_joint_angle[4] = 0.0
                path_time = 5.0
                teleop_keyboard.send_goal_joint_space()
            elif key_value == '5':
                goal_joint_angle[4] = -0.01
                prev_goal_joint_angle[4] = goal_joint_angle[4]
                teleop_keyboard.send_tool_control_request()
            elif key_value == '6':
                goal_joint_angle[0] = 0.0
                goal_joint_angle[1] = 0.0
                goal_joint_angle[2] = 0.0
                goal_joint_angle[3] = 1.3
                goal_joint_angle[4] = 0.0
                path_time = 5.0
                teleop_keyboard.send_goal_joint_space()
            else:
                if key_value == '\x03':
                    break
                else:
                    for index in range(0, 7):
                        prev_goal_kinematics_pose[index] = goal_kinematics_pose[index]
                    for index in range(0, 5):
                        prev_goal_joint_angle[index] = goal_joint_angle[index]

    except Exception as e:
        print(e)

    finally:
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        teleop_keyboard.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

