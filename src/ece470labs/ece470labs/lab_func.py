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
	# end effector frame relative to base frame, x y z values are last column bottom row is 0,0,0,1 for movement purpsoes
	M = np.array([[0,-1,0,-150 + 244 + 213 + 85],
				[0,0,-1,150 + 120 -93 + 104 + 92 + 59],
				[1,0,0,53.5 + 152 ],
				[0,0,0,1]])
	S = np.array([[0,0,1,0,0,0],
					[0,1,0,0,0,0],
					[0,1,0,0,0,0],
					[0,1,0,0,0,0],
					[1,0,0,0,0,0],
					[0,1,0,0,0,0]]).T
	pos = np.array([[-150, 150, 0], [-150, 270, 152], [94, 270, 152],[307,177,152],[307, 281, 152],[392,281,152]])
	S[3:, :] = -np.cross(S[:3, :].T, pos).T
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
	T = np.eye(4)
	thetas = np.array([theta1, theta2, theta3, theta4, theta5, theta6]) 
	M,S = Get_MS()
	for i in range(S.shape[1]):
		
		S_col = S[:, i]
		
		w = S_col[:3]
		v = S_col[3:]
		

		w_skew = np.array([
			[ 0,     -w[2],  w[1]],
			[ w[2],   0,    -w[0]],
			[-w[1],   w[0],  0   ]
		])
		
		se3_matrix = np.zeros((4, 4))
		se3_matrix[:3, :3] = w_skew
		se3_matrix[:3, 3] = v
	
		T_joint = expm(se3_matrix * thetas[i])
		T = T @ T_joint

	T_final = T @ M

	print(str(T_final) + "\n")

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
	
	L3 = 244
	L5 = 213
	L7 = 85
	L9 = 53.5
	
	drop = 92 + 59
	lateral = 120 - 93 + 104
	shoulder_z = 152
	
	xgrip = float(xWgrip) * 1000 + 150
	ygrip = float(yWgrip) * 1000 - 150
	zgrip = float(zWgrip) * 1000
	
	yaw = math.radians(float(yaw_WgripDegree))
	
	xcen = xgrip - L9 * math.cos(yaw)
	ycen = ygrip - L9 * math.sin(yaw)
	zcen = zgrip
	
	radius = math.hypot(xcen, ycen)
	
	if radius < lateral - 1e-9:
	    raise ValueError("Unreachable XY point")
	
	correction = math.asin(
	    float(np.clip(lateral / radius, -1.0, 1.0))
	)
	
	theta1 = math.atan2(ycen, xcen) - correction
	
	theta6 = theta1 + pi / 2.0 - yaw
	theta6 = (theta6 + pi) % (2.0 * pi) - pi
	
	x3end = (
	    xcen
	    - L7 * math.cos(theta1)
	    + lateral * math.sin(theta1)
	)
	
	y3end = (
	    ycen
	    - L7 * math.sin(theta1)
	    - lateral * math.cos(theta1)
	)
	
	z3end = zcen + drop
	
	r = (
	    x3end * math.cos(theta1)
	    + y3end * math.sin(theta1)
	)
	
	h = z3end - shoulder_z
	
	cos_theta3 = (
	    r**2 + h**2 - L3**2 - L5**2
	) / (2.0 * L3 * L5)
	
	if not -1.0 - 1e-9 <= cos_theta3 <= 1.0 + 1e-9:
	    raise ValueError("Unreachable point")
	
	cos_theta3 = float(np.clip(cos_theta3, -1.0, 1.0))
	
	theta3 = math.acos(cos_theta3)
	
	theta2 = (
	    math.atan2(-h, r)
	    - math.atan2(
	        L5 * math.sin(theta3),
	        L3 + L5 * math.cos(theta3)
	    )
	)
	
	theta4 = -(theta2 + theta3)
	
	theta5 = -pi / 2.0
		
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
