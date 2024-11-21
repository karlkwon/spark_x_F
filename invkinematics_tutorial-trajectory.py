import numpy as np
from visual_kinematics.RobotSerial import *
from visual_kinematics.RobotTrajectory import *
from math import pi

# d, a, alpha, theta
dh_params2 = np.array([[0.089, 0., 0.5 * pi, 0.],
                      [0., -0.425, 0, 0.],
                      [0., -0.392, 0, 0.],
                      [0.109, 0., 0.5 * pi, 0.],
                      [0.095, 0., -0.5 * pi, 0.],
                      [0.082, 0., 0., 0.]])
robot = RobotSerial(dh_params2)

# robot.show()

frames = [Frame.from_euler_3(np.array([0., 0., 0.]), np.array([[0.28127], [0.], [0.5]])),
          Frame.from_euler_3(np.array([0., 0., 0.25 * pi]), np.array([[0.28127], [0.], [0.5]])),
          Frame.from_euler_3(np.array([0, 0., pi]), np.array([[0.28127], [0.], [0.5]]))]

trajectory = RobotTrajectory(robot, frames)
trajectory.show(motion="p2p")