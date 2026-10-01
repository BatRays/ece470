#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from math import pi
import math

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
<<<<<<< HEAD
	M = np.eye(4)
	S = np.zeros((6,6))
 
=======
	# end effector frame relative to base frame, x y z values are last column bottom row is 0,0,0,1 for movement purpsoes
	M = np.array([[0,0,-1,152 + 244 + 213 + 85 - 150],[-1,0,0,150 + 120 -93 + 104 + 92 + 59],[0,-1,0,215.5],[0,0,0,1]])
	S = np.array([[0,0,1,0,0,150],[0,1,0,150,0,0],[0,1,0,0,0,0],[0,1,0,0,0,0],[1,0,0,0,0,0],[0,1,0,0,0,0]])
	pos = np.array([-150, 150, 0], [-150 ,270,162], [94, 270, 162],[309, 177, 162][-150 + 152 + 244 + 213,150 + 120 - 93 + 104,])
>>>>>>> 580b939 (Lab 3 End Effector)

	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#

	T = [ [1.0, 0.0, 0.0], \
      [0.0, 1.0, 0.0], \
      [0.0, 0.0, 1.0] ]
	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + pi
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*pi)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	
	theta1 = 0.0
	theta2 = 0.0
	theta3 = 0.0
	theta4 = 0.0
	theta5 = 0.0
	theta6 = 0.0
	
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
