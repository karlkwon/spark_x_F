import numpy as np
from visual_kinematics.RobotSerial import *
from math import pi

# d, a, alpha, theta
dh_params2 = np.array([[0.089, 0., 0.5 * pi, 0.],
                      [0., -0.425, 0, 0.],
                      [0., -0.392, 0, 0.],
                      [0.109, 0., 0.5 * pi, 0.],
                      [0.095, 0., -0.5 * pi, 0.],
                      [0.082, 0., 0., 0.]])
robot = RobotSerial(dh_params2)

robot.show()
