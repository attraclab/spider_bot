import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
import time

np.set_printoptions(suppress=True)

class SpiderBotLib:

	def __init__(self):

		#####################
		## Legs parameters ##
		#####################
		# L1 : Coxa link
		# L2 : Femur link
		# L3 : Tibia link
		self.L1 = 42.25
		self.L2 = 120.0
		self.L3 = 180.0

		###################
		## Home position ##
		###################
		# a starting position (rest position) before start walking
		# theta1,2,3_home are from forward kinematics, or kinematics diagram in Solidworks
		self.X_home = 170.0
		self.Y_home = 0.0
		self.Z_home = -100.0


		# self.theta1_home = 0.0
		# self.theta2_home = np.radians(14.13)
		# self.theta3_home = np.radians(-96.59)
		self.theta1_home, self.theta2_home, self.theta3_home = self.inv(self.X_home, self.Y_home, self.Z_home)

		self.XYZ_home = np.array([self.X_home, self.Y_home, self.Z_home])

		########################
		## Walking parameters ##
		########################
		# S : starting point offset from 0
		# T : step distance (how far the foot will move)
		# A : step height (how the foot will lift from ground)
		# beta : angle between ci and cj (1st leg and 2nd leg)
		# x_start : X starting position before walking
		# data_point_per_line : how many data point to generate for line or curve walking path

		self.S = self.Z_home #self.Z_home
		self.T = 200.0 #200
		self.A = 80 #80
		self.beta = 57.53
		self.beta_rad = np.radians(self.beta)
		self.x_start = self.X_home #160

		self.data_point_per_line = 20
		self.DATA_POINT_ALL = int(self.data_point_per_line*2)
		## X_line and X_curve will start from 0, once we apply rotation matrix of each leg
		## then we add x_start to X matrix to translate rotated matrices to starting point
		self.Y_line = np.linspace(self.T/2, -self.T/2, self.data_point_per_line)
		self.Z_line = np.linspace(self.S, self.S, self.data_point_per_line)
		self.X_line = np.linspace(0.0, 0.0, self.data_point_per_line)

		## Bezier's curve
		P1 = [-self.T/2, self.S]
		P2 = [0, (self.S+(2*self.A))]
		P3 = [self.T/2, self.S]
		t = np.linspace(0,1, self.data_point_per_line)

		self.Y_curve = (((1-t)**2)*P1[0]) + 2*(1-t)*t*P2[0] + (t**2)*P3[0]
		self.Z_curve = (((1-t)**2)*P1[1]) + 2*(1-t)*t*P2[1] + (t**2)*P3[1]
		self.X_curve = np.copy(self.X_line)

		## rotation angle to rotation foot path of each leg
		## the line+curve points have to rotate according to each leg
		self.walk_rot_ang_1 = 0.0
		self.walk_rot_ang_2 = np.radians(-self.beta)
		self.walk_rot_ang_3 = np.radians(self.beta)
		self.walk_rot_ang_4 = np.radians(180)
		self.walk_rot_ang_5 = np.radians(-self.beta)
		self.walk_rot_ang_6 = np.radians(self.beta)

		## Crab-walking ##
		# 12 is 360/30 so we have an increment of 30deg as 12 step resolution
		# 18 is the PWM1_1, PWM2_1, PWM3_1, ...., PWM1_6, PWM2_6, PWM3_6 total as 18 array of PWM
		self.cw_ang_resolution = 10 #30
		self.cw_LUT_increment = 360//self.cw_ang_resolution
		self.crab_walking_LUT_THETA = np.empty((self.cw_LUT_increment, 18, self.DATA_POINT_ALL))

		###################################
		## Custome Pose Crab-Walking LUT ##
		###################################
		self.S_cus = [self.Z_home, self.Z_home, self.Z_home, self.Z_home, self.Z_home, self.Z_home]
		self.T_cus = [self.T/2,self.T/2,self.T/2,self.T/2,self.T/2,self.T/2]
		self.A_cus = [50,50,50,50,50,50]
		self.x_start_cus = [self.X_home, self.X_home, self.X_home, self.X_home, self.X_home, self.X_home]

		self.crab_walking_LUT_custom_THETA = np.empty((self.cw_LUT_increment, 18, self.DATA_POINT_ALL))

		###############
		## Wave gait ##
		###############
		self.wg_data_per_blk = 10
		self.WG_DATA_POINT_ALL = int(self.wg_data_per_blk*6)
		self.wg_ang_resolution = 10
		self.wg_LUT_increment = 360//self.wg_ang_resolution
		self.wave_gait_LUT_THETA = np.empty((self.wg_LUT_increment, 18, self.WG_DATA_POINT_ALL))

		#####################
		## Body parameters ##
		#####################
		
		# C1 : distance between center point C and leg i frame
		# C2 : distance between center point C and leg j frame
		# PC_start : initial point of point C, should be 0,0,0	
		self.C1 = 107.25
		self.C2 = 145.635
		self.PC_start = np.array([0,0,0])

		#############################################
		#### Body Translation & Rotaiton control ####
		#############################################
		### Translation ###
		## when center point of body translates in xyz plane
		## we need to apply this body rotation according to each leg
		self.leg1_ang_offset = 0.0
		self.leg2_ang_offset = np.radians(-self.beta)
		self.leg3_ang_offset = np.radians(-(180-self.beta))
		self.leg4_ang_offset = np.radians(-180)
		self.leg5_ang_offset = np.radians(180-self.beta)
		self.leg6_ang_offset = np.radians(self.beta)

		self.leg1_offset_rot = np.array([[np.cos(self.leg1_ang_offset), -np.sin(self.leg1_ang_offset), 0],
									[np.sin(self.leg1_ang_offset), np.cos(self.leg1_ang_offset), 0], 
									[0, 0, 1]])
		self.leg2_offset_rot = np.array([[np.cos(self.leg2_ang_offset), -np.sin(self.leg2_ang_offset), 0],
									[np.sin(self.leg2_ang_offset), np.cos(self.leg2_ang_offset), 0], 
									[0, 0, 1]])
		self.leg3_offset_rot = np.array([[np.cos(self.leg3_ang_offset), -np.sin(self.leg3_ang_offset), 0],
									[np.sin(self.leg3_ang_offset), np.cos(self.leg3_ang_offset), 0], 
									[0, 0, 1]])
		self.leg4_offset_rot = np.array([[np.cos(self.leg4_ang_offset), -np.sin(self.leg4_ang_offset), 0],
									[np.sin(self.leg4_ang_offset), np.cos(self.leg4_ang_offset), 0], 
									[0, 0, 1]])
		self.leg5_offset_rot = np.array([[np.cos(self.leg5_ang_offset), -np.sin(self.leg5_ang_offset), 0],
									[np.sin(self.leg5_ang_offset), np.cos(self.leg5_ang_offset), 0], 
									[0, 0, 1]])
		self.leg6_offset_rot = np.array([[np.cos(self.leg6_ang_offset), -np.sin(self.leg6_ang_offset), 0],
									[np.sin(self.leg6_ang_offset), np.cos(self.leg6_ang_offset), 0], 
									[0, 0, 1]])

		### Rotation ###
		## translation offset from frame O to leg frame
		self.trans_O_to_leg1 = np.array([-self.C1, 0, 0])
		self.trans_O_to_leg2 = np.array([-self.C2, 0, 0])
		self.trans_O_to_leg3 = np.array([-self.C2, 0, 0])
		self.trans_O_to_leg4 = np.array([-self.C1, 0, 0])
		self.trans_O_to_leg5 = np.array([-self.C2, 0, 0])
		self.trans_O_to_leg6 = np.array([-self.C2, 0, 0])

		## XYZ home position of each leg in O frame
		self.XYZ_home_OI = np.array([(self.X_home+self.C1), 0, self.Z_home])
		self.XYZ_home_OJ = np.array([(self.X_home+self.C2)*np.cos(self.beta_rad), (self.X_home+self.C2)*np.sin(self.beta_rad), self.Z_home])
		self.XYZ_home_OK = np.array([-(self.X_home+self.C2)*np.cos(self.beta_rad), (self.X_home+self.C2)*np.sin(self.beta_rad), self.Z_home])
		self.XYZ_home_OL = np.array([-(self.X_home+self.C1), 0, self.Z_home])
		self.XYZ_home_OM = np.array([-(self.X_home+self.C2)*np.cos(self.beta_rad), -(self.X_home+self.C2)*np.sin(self.beta_rad), self.Z_home])
		self.XYZ_home_ON = np.array([(self.X_home+self.C2)*np.cos(self.beta_rad), -(self.X_home+self.C2)*np.sin(self.beta_rad), self.Z_home])

		#######################
		## In-plance turning ##
		#######################
		## assume leg i,k,m are already lifted up
		self.data_point_TURN_per_blk = 10
		self.data_point_TURN_ALL = self.data_point_TURN_per_blk*6
		x_LU_last = self.X_home
		y_LU_last = self.Y_home
		z_LU_last = -50
		self.xyz_LU_last = np.array([x_LU_last, y_LU_last, z_LU_last])

		x_LD_last = self.X_home
		y_LD_last = self.Y_home
		z_LD_last = self.Z_home
		self.xyz_LD_last = np.array([x_LD_last, y_LD_last, z_LD_last])

		## from lift-up last and keep holding
		self.xyz_LU_hold = np.linspace(self.xyz_LU_last, self.xyz_LU_last, self.data_point_TURN_per_blk)
		## from lift-up hold to lift-down, this is lift-down moves
		self.xyz_LD_move = np.linspace(self.xyz_LU_last, self.xyz_LD_last, self.data_point_TURN_per_blk)
		## from lift-down last keep holding
		self.xyz_LD_hold = np.linspace(self.xyz_LD_last, self.xyz_LD_last, self.data_point_TURN_per_blk)

		self.theta2_drag = np.linspace(self.theta2_home, self.theta2_home, self.data_point_TURN_per_blk)
		self.theta3_drag = np.linspace(self.theta3_home, self.theta3_home, self.data_point_TURN_per_blk)

		self.xyz_drag_move = np.empty((self.data_point_TURN_per_blk, 3), dtype=np.float32)

		################################
		## Normalk walling & Steering ##
		################################
		self.curve_path = self.T
		self.leg_STR_Z_line = np.linspace(self.S, self.S, self.data_point_per_line)
		self.R_icc_max = 3000
		self.R_icc_min = 600
		self.normWalking_LUT_increment = 15
		##    steer CCW    str_sign = 1.0    <----      ---->  steer CW str_sign = -1.0
		## R_icc  600 1000 1400 1800 2200 2600 3000 [] 3000 2600 2200 1800 1400 1000 600  
		## Index    0    1    2    3    4   5   6    7    8    9   10   11   12   13  14
		self.normWalking_LUT_THETA = np.empty((self.normWalking_LUT_increment, 18, self.DATA_POINT_ALL)) 
		self.normWalking_LUT_PWM = np.empty((self.normWalking_LUT_increment, 18, self.DATA_POINT_ALL)) 


	def fwd(self, theta1, theta2, theta3):
		"""
		Forward kinetmaics on single leg frame
			
		Parameters:
			theta1, theta2, theta3: angle on of each joint in radians
		
		Returns:
			x,y,z: foot tip position on cartesian coordinate in mm
		"""

		x = self.L1*np.cos(theta1) + (self.L2*np.cos(theta2) + self.L3*np.cos(theta2+theta3))*np.cos(theta1)
		y = self.L1*np.sin(theta1) + (self.L2*np.cos(theta2) + self.L3*np.cos(theta2+theta3))*np.sin(theta1)
		z = self.L2*np.sin(theta2) + self.L3*np.sin(theta2+theta3)

		return x,y,z

	def inv(self, xp, yp, zp):
		"""
		Inverse kinematics on single leg frame

		Parameters:
			xp, yp, zp: desired foot tip on cartesian coordinates in mm

		Returns:
			theta1, theta2, theta3: each joint angle in radians
		"""

		theta1 = np.arctan(yp/xp)

		r2 = xp/np.cos(theta1) - self.L1
		phi2 = np.arctan(zp/r2)
		r1 = np.sqrt(r2**2 + zp**2)
		phi1 = np.arccos(-((self.L3**2 - self.L2**2 - r1**2)/(2*self.L2*r1)))

		## leg down, so phi2 already negative
		theta2 = phi1 + phi2

		phi3 = np.arccos(-(r1**2 - self.L2**2 - self.L3**2)/(2*self.L2*self.L3))
		theta3 = -(pi - phi3)

		return theta1, theta2, theta3

	def invKinArray_to_ThetaArray(self, XYZ_array):
		"""
		Doing inverse kinematics from array of input

		Parameters:
			XYZ_array: numpy array in the form of [[x1,y1,z1], [x2,y2,z2], [x3,y3,z3], ..., [xN, yN, zN] ] unit is in mm

		Rerturns:
			THETA1: theta1 set array [theta1_1, theta1_2, theta1_3, theta1_4, theta1_5, ..., theta1_N]
			THETA2: theta2 set array [theta2_1, theta2_2, theta2_3, theta2_4, theta2_5, ..., theta2_N]
			THETA3: theta3 set array [theta3_1, theta3_2, theta3_3, theta3_4, theta3_5, ..., theta3_N]
		"""
		THETA1 = np.array([])
		THETA2 = np.array([])
		THETA3 = np.array([])

		for x,y,z in XYZ_array:

			theta1, theta2, theta3 = self.inv(x, y, z)
			THETA1 = np.append(THETA1, theta1)
			THETA2 = np.append(THETA2, theta2)
			THETA3 = np.append(THETA3, theta3)

		return THETA1, THETA2, THETA3

	def map_with_limit(self, val, in_min, in_max, out_min, out_max):
		"""
		Mapping one value range to another range with minimum and maximum limit

		"""

		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min

		if out_min > out_max:
			if out > out_min:
				out = out_min
			elif out < out_max:
				out = out_max
			else:
				pass
		elif out_max > out_min:
			if out > out_max:
				out = out_max
			elif out < out_min:
				out = out_min
			else:
				pass
		else:
			pass

		return out

	def XYZ_TriGait_gen(self, leg_id, dir_rot_ang):
		"""
		Generate XYZ points (path of dragging + lifting motion) of each leg
		The motion we are using here is tri-gait 
		so there are three legs always touch the ground 
		and three legs lift up in the air

		Parameters:
			leg_id: leg's number 1, 2, 3, 4, 5, or 6
			dir_rot_ang: direction angle of robot's facing

		Returns:
			XYZ_R[0]: array of X values
			XYZ_R[1]: array of Y values
			XYZ_R[2]: array of Z values
		"""

		if leg_id == 1:
			rot = self.walk_rot_ang_1 + np.radians(dir_rot_ang)
		elif leg_id == 2:
			rot = self.walk_rot_ang_2 + np.radians(dir_rot_ang)
		elif leg_id == 3:
			rot = self.walk_rot_ang_3 + np.radians(dir_rot_ang)
		elif leg_id == 4:
			rot = self.walk_rot_ang_4 + np.radians(dir_rot_ang)
		elif leg_id == 5:
			rot = self.walk_rot_ang_5 + np.radians(dir_rot_ang)
		elif leg_id == 6:
			rot = self.walk_rot_ang_6 + np.radians(dir_rot_ang)


		if (leg_id == 1):

			X_line_1st_half = self.X_line[:len(self.X_line)//2]
			X_line_2nd_half = self.X_line[len(self.X_line)//2:]
			Y_line_1st_half = self.Y_line[:len(self.Y_line)//2]
			Y_line_2nd_half = self.Y_line[len(self.Y_line)//2:]
			Z_line_1st_half = self.Z_line[:len(self.Z_line)//2]
			Z_line_2nd_half = self.Z_line[len(self.Z_line)//2:]

			X = np.concatenate((X_line_2nd_half, self.X_curve, X_line_1st_half), axis=0)
			Y = np.concatenate((Y_line_2nd_half, self.Y_curve, Y_line_1st_half), axis=0)
			Z = np.concatenate((Z_line_2nd_half, self.Z_curve, Z_line_1st_half), axis=0)


		elif (leg_id == 2) or (leg_id == 6) or (leg_id == 4):

			X_curve_1st_half = self.X_curve[:len(self.X_curve)//2]
			X_curve_2nd_half = self.X_curve[len(self.X_curve)//2:]
			Y_curve_1st_half = self.Y_curve[:len(self.Y_curve)//2]
			Y_curve_2nd_half = self.Y_curve[len(self.Y_curve)//2:]
			Z_curve_1st_half = self.Z_curve[:len(self.Z_curve)//2]
			Z_curve_2nd_half = self.Z_curve[len(self.Z_curve)//2:]

			X = np.concatenate((X_curve_2nd_half, self.X_line, X_curve_1st_half), axis=0)
			Y = np.concatenate((Y_curve_2nd_half, self.Y_line, Y_curve_1st_half), axis=0)
			Z = np.concatenate((Z_curve_2nd_half, self.Z_line, Z_curve_1st_half), axis=0)


		elif (leg_id == 3) or (leg_id == 5):

			X_line_tmp = np.flip(self.X_line, 0)
			Y_line_tmp = np.flip(self.Y_line, 0)
			Z_line_tmp = np.flip(self.Z_line, 0)

			X_curve_tmp = np.flip(self.X_curve, 0)
			Y_curve_tmp = np.flip(self.Y_curve, 0)
			Z_curve_tmp = np.flip(self.Z_curve, 0)

			X_line_1st_half = X_line_tmp[:len(X_line_tmp)//2]
			X_line_2nd_half = X_line_tmp[len(X_line_tmp)//2:]
			Y_line_1st_half = Y_line_tmp[:len(Y_line_tmp)//2]
			Y_line_2nd_half = Y_line_tmp[len(Y_line_tmp)//2:]
			Z_line_1st_half = Z_line_tmp[:len(Z_line_tmp)//2]
			Z_line_2nd_half = Z_line_tmp[len(Z_line_tmp)//2:]

			X = np.concatenate((X_line_2nd_half, X_curve_tmp, X_line_1st_half), axis=0)
			Y = np.concatenate((Y_line_2nd_half, Y_curve_tmp, Y_line_1st_half), axis=0)
			Z = np.concatenate((Z_line_2nd_half, Z_curve_tmp, Z_line_1st_half), axis=0)


		### rotation matrix of rot matrix ###
		ROT = np.array([[np.cos(rot), -np.sin(rot), 0], 
						[np.sin(rot), np.cos(rot), 0], 
						[0, 0, 1]])

		XYZ = np.stack((X,Y,Z), axis=-1)

		### transform XYZ of (0,0,0) frame to each leg frame ###
		XYZ_R = np.array([])
		for i in range(len(XYZ)):

			xyz_r = np.matmul(ROT, XYZ[i].T)
			xyz_r = np.expand_dims(xyz_r, axis=0)
			if i == 0:
				XYZ_R = np.append(XYZ_R, xyz_r)
				XYZ_R = np.expand_dims(XYZ_R, axis=0)
			else:
				XYZ_R = np.concatenate((XYZ_R, xyz_r))

		XYZ_R[:,0] = XYZ_R[:,0] + self.x_start

		XYZ_R = np.transpose(XYZ_R)

		return XYZ_R[0], XYZ_R[1], XYZ_R[2]


	def XYZ_TriGait_gen_custom(self, leg_id, dir_rot_ang, body_pitch, x_start, z_start):
		"""
		Generate XYZ points (path of dragging + lifting motion) of each leg
		The motion we are using here is tri-gait 
		so there are three legs always touch the ground 
		and three legs lift up in the air

		Parameters:
			leg_id: leg's number 1, 2, 3, 4, 5, or 6
			dir_rot_ang: direction angle of robot's facing

		Returns:
			XYZ_R[0]: array of X values
			XYZ_R[1]: array of Y values
			XYZ_R[2]: array of Z values
		"""

		S = 0.0
		T = 100.0 #200
		A = 80 #80
		P1 = [-T/2, S]
		P2 = [0, (2*A)]
		P3 = [T/2, 0.0]
		t = np.linspace(0,1, self.data_point_per_line)

		X_line = np.linspace(0,0, self.data_point_per_line)
		Y_line = np.linspace(T/2, -T/2, self.data_point_per_line)
		Z_line = np.linspace(0, 0, self.data_point_per_line)

		Y_curve = (((1-t)**2)*P1[0]) + 2*(1-t)*t*P2[0] + (t**2)*P3[0]
		Z_curve = (((1-t)**2)*P1[1]) + 2*(1-t)*t*P2[1] + (t**2)*P3[1]
		X_curve = np.copy(X_line)

		
		if (leg_id == 1):


			X_line_1st_half = X_line[:len(X_line)//2]
			X_line_2nd_half = X_line[len(X_line)//2:]
			Y_line_1st_half = Y_line[:len(Y_line)//2]
			Y_line_2nd_half = Y_line[len(Y_line)//2:]
			Z_line_1st_half = Z_line[:len(Z_line)//2]
			Z_line_2nd_half = Z_line[len(Z_line)//2:]

			X = np.concatenate((X_line_2nd_half, X_curve, X_line_1st_half), axis=0)
			Y = np.concatenate((Y_line_2nd_half, Y_curve, Y_line_1st_half), axis=0)
			Z = np.concatenate((Z_line_2nd_half, Z_curve, Z_line_1st_half), axis=0)


		elif (leg_id == 2) or (leg_id == 6) or (leg_id == 4):

			X_curve_1st_half = X_curve[:len(X_curve)//2]
			X_curve_2nd_half = X_curve[len(X_curve)//2:]
			Y_curve_1st_half = Y_curve[:len(Y_curve)//2]
			Y_curve_2nd_half = Y_curve[len(Y_curve)//2:]
			Z_curve_1st_half = Z_curve[:len(Z_curve)//2]
			Z_curve_2nd_half = Z_curve[len(Z_curve)//2:]

			X = np.concatenate((X_curve_2nd_half, X_line, X_curve_1st_half), axis=0)
			Y = np.concatenate((Y_curve_2nd_half, Y_line, Y_curve_1st_half), axis=0)
			Z = np.concatenate((Z_curve_2nd_half, Z_line, Z_curve_1st_half), axis=0)


		elif (leg_id == 3) or (leg_id == 5):

			X_line_tmp = np.flip(X_line, 0)
			Y_line_tmp = np.flip(Y_line, 0)
			Z_line_tmp = np.flip(Z_line, 0)

			X_curve_tmp = np.flip(X_curve, 0)
			Y_curve_tmp = np.flip(Y_curve, 0)
			Z_curve_tmp = np.flip(Z_curve, 0)

			X_line_1st_half = X_line_tmp[:len(X_line_tmp)//2]
			X_line_2nd_half = X_line_tmp[len(X_line_tmp)//2:]
			Y_line_1st_half = Y_line_tmp[:len(Y_line_tmp)//2]
			Y_line_2nd_half = Y_line_tmp[len(Y_line_tmp)//2:]
			Z_line_1st_half = Z_line_tmp[:len(Z_line_tmp)//2]
			Z_line_2nd_half = Z_line_tmp[len(Z_line_tmp)//2:]

			X = np.concatenate((X_line_2nd_half, X_curve_tmp, X_line_1st_half), axis=0)
			Y = np.concatenate((Y_line_2nd_half, Y_curve_tmp, Y_line_1st_half), axis=0)
			Z = np.concatenate((Z_line_2nd_half, Z_curve_tmp, Z_line_1st_half), axis=0)

		# print("leg_id", leg_id)
		# print("X", X)
		# print("Y", Y)
		# print("Z", Z)

		if leg_id == 1:
			rot = self.walk_rot_ang_1 + np.radians(dir_rot_ang)
		elif leg_id == 2:
			rot = self.walk_rot_ang_2 + np.radians(dir_rot_ang)
		elif leg_id == 3:
			rot = self.walk_rot_ang_3 + np.radians(dir_rot_ang)
		elif leg_id == 4:
			rot = self.walk_rot_ang_4 + np.radians(dir_rot_ang)
		elif leg_id == 5:
			rot = self.walk_rot_ang_5 + np.radians(dir_rot_ang)
		elif leg_id == 6:
			rot = self.walk_rot_ang_6 + np.radians(dir_rot_ang)

		### rotation matrix of rot matrix ###
		ROT_yaw = np.array([[np.cos(rot), -np.sin(rot), 0], 
						[np.sin(rot), np.cos(rot), 0], 
						[0, 0, 1]])

		ROT_p = np.array([	[np.cos(0.0), 0, np.sin(0.0)],
								[0, 1, 0],
								[-np.sin(0.0), 0, np.cos(0.0)]])

		ROT_r = np.array([	[1, 0, 0],
								[0, np.cos(body_pitch), -np.sin(body_pitch)],
								[0, np.sin(body_pitch), np.cos(body_pitch)]])

		ROT = np.matmul(np.matmul(ROT_yaw, ROT_p), ROT_r)
		# ROT = np.matmul(np.matmul(ROT_r, ROT_p), ROT_yaw)
		# ROT = ROT_yaw



		XYZ = np.stack((X,Y,Z), axis=-1)

		### transform XYZ of (0,0,0) frame to each leg frame ###
		XYZ_R = np.array([])
		for i in range(len(XYZ)):

			xyz_r = np.matmul(ROT, XYZ[i].T)
			xyz_r = np.expand_dims(xyz_r, axis=0)
			if i == 0:
				XYZ_R = np.append(XYZ_R, xyz_r)
				XYZ_R = np.expand_dims(XYZ_R, axis=0)
			else:
				XYZ_R = np.concatenate((XYZ_R, xyz_r))

		## shift X
		XYZ_R[:,0] = XYZ_R[:,0] + x_start
		## shift Z
		XYZ_R[:,2] = XYZ_R[:,2] + z_start

		XYZ_R = np.transpose(XYZ_R)

		return XYZ_R[0], XYZ_R[1], XYZ_R[2]

	def transformation_from_cFrame_to_legFrame(self, leg_id, PC_new):

		## leg frame is static, center body frame is moving

		## Get translation matrix from leg to center body frame
		if (leg_id == 1) or (leg_id == 4):
			trans_cFrame_to_legFrame = np.array([-self.C1, 0, 0])
		else:
			trans_cFrame_to_legFrame = np.array([-self.C2, 0, 0])

		## Get rotation matrix from leg frame to center frame
		## rotation angle is measured by leg static frame to center body frame
		if leg_id == 1:
			rot_ang = self.body_rot_ang_1
		elif leg_id == 2:
			rot_ang = self.body_rot_ang_2
		elif leg_id == 3:
			rot_ang = self.body_rot_ang_3
		elif leg_id == 4:
			rot_ang = self.body_rot_ang_4
		elif leg_id == 5:
			rot_ang = self.body_rot_ang_5
		elif leg_id == 6:
			rot_ang = self.body_rot_ang_6

		rot_cFrame_to_legFrame = np.array([[np.cos(rot_ang), -np.sin(rot_ang), 0],
											[np.sin(rot_ang), np.cos(rot_ang), 0], 
											[0, 0, 1]])

		## PC starting point in leg frame
		## PC_start is in center body frame, we apply rotation matrix to convert it to leg frame
		## PC_start_legFrame is PC_start point in leg frame
		PC_start_legFrame = np.matmul(rot_cFrame_to_legFrame, self.PC_start) + trans_cFrame_to_legFrame

		## get PC_new in leg frame
		PC_new_legFrame = np.matmul(rot_cFrame_to_legFrame, PC_new) + trans_cFrame_to_legFrame

		## a difference or changed of translation from PC_new in leg frame
		PC_trans_changed = PC_start_legFrame - PC_new_legFrame

		return PC_trans_changed

	def bodyTranslate_to_newLegXYZ(self, PC_new):
		"""
		Convert a center point position (Xc, Yc, Zc) of body frame 
		to XYZ point of foot tip of each leg

		Parameters:
			PC_new: numpy array of center point position [Xc, Yc, Zc]

		Returns:
			leg1_newXYZ: foot tip position [x1, y1, z1] of leg 1 in order to make body move to PC_new 
			leg2_newXYZ: foot tip position [x2, y2, z2] of leg 2 in order to make body move to PC_new
			leg3_newXYZ: foot tip position [x3, y3, z3] of leg 3 in order to make body move to PC_new
			leg4_newXYZ: foot tip position [x4, y4, z4] of leg 4 in order to make body move to PC_new
			leg5_newXYZ: foot tip position [x5, y5, z5] of leg 5 in order to make body move to PC_new
			leg6_newXYZ: foot tip position [x6, y6, z6] of leg 6 in order to make body move to PC_new
		"""

		## Simplified ##
		leg1_newXYZ = self.XYZ_home - np.matmul(self.leg1_offset_rot, PC_new)
		leg2_newXYZ = self.XYZ_home - np.matmul(self.leg2_offset_rot, PC_new)
		leg3_newXYZ = self.XYZ_home - np.matmul(self.leg3_offset_rot, PC_new)
		leg4_newXYZ = self.XYZ_home - np.matmul(self.leg4_offset_rot, PC_new)
		leg5_newXYZ = self.XYZ_home - np.matmul(self.leg5_offset_rot, PC_new)
		leg6_newXYZ = self.XYZ_home - np.matmul(self.leg6_offset_rot, PC_new)

		return leg1_newXYZ, leg2_newXYZ, leg3_newXYZ, leg4_newXYZ, leg5_newXYZ, leg6_newXYZ

	def get_RPY_matrix(self, r, p, y, sign):
		"""
		Generate rotation matrix

		Parameters:
			r, p, y: roll, pitch, yaw angles in radians
			sign: +1 or -1 sign

		Returns:
			result: 3x3 rotation matrix (numpy array)
		"""

		r = r*sign
		p = p*sign
		y = y*sign

		rot_y = np.array([	[np.cos(y), -np.sin(y), 0],
						[np.sin(y), np.cos(y), 0],
						[0, 0, 1]])

		rot_p = np.array([	[np.cos(p), 0, np.sin(p)],
								[0, 1, 0],
								[-np.sin(p), 0, np.cos(p)]])

		rot_r = np.array([	[1, 0, 0],
								[0, np.cos(r), -np.sin(r)],
								[0, np.sin(r), np.cos(r)]])


		result = np.matmul(np.matmul(rot_y, rot_p), rot_r)
	

		return result #np.matmul(rot_y, rot_p, rot_r)


	def bodyRotate_to_newLegXYZ(self, r, p, y):
		"""
		Convert roll, pitch, yaw of body in body frame to 
		how much of each leg postion has to move

		Parameters:
			r, p, y: roll, pitch, yaw angles of body

		Returns:
			leg1_newXYZ: foot tip position [x1, y1, z1] of leg 1 in order to make body rotate as r,p,y angles 
			leg2_newXYZ: foot tip position [x2, y2, z2] of leg 2 in order to make body rotate as r,p,y angles
			leg3_newXYZ: foot tip position [x3, y3, z3] of leg 3 in order to make body rotate as r,p,y angles
			leg4_newXYZ: foot tip position [x4, y4, z4] of leg 4 in order to make body rotate as r,p,y angles
			leg5_newXYZ: foot tip position [x5, y5, z5] of leg 5 in order to make body rotate as r,p,y angles
			leg6_newXYZ: foot tip position [x6, y6, z6] of leg 6 in order to make body rotate as r,p,y angles
		"""

		
		## Reverse angle body rotation matrix
		REV_ROT_MUL = self.get_RPY_matrix(r,p,y, -1.0)

		rot_O_to_leg1 = np.matmul(self.leg1_offset_rot, REV_ROT_MUL)
		rot_O_to_leg2 = np.matmul(self.leg2_offset_rot, REV_ROT_MUL)
		rot_O_to_leg3 = np.matmul(self.leg3_offset_rot, REV_ROT_MUL)
		rot_O_to_leg4 = np.matmul(self.leg4_offset_rot, REV_ROT_MUL)
		rot_O_to_leg5 = np.matmul(self.leg5_offset_rot, REV_ROT_MUL)
		rot_O_to_leg6 = np.matmul(self.leg6_offset_rot, REV_ROT_MUL)

		leg1_newXYZ = np.matmul(rot_O_to_leg1, self.XYZ_home_OI) + self.trans_O_to_leg1
		leg2_newXYZ = np.matmul(rot_O_to_leg2, self.XYZ_home_OJ) + self.trans_O_to_leg2
		leg3_newXYZ = np.matmul(rot_O_to_leg3, self.XYZ_home_OK) + self.trans_O_to_leg3
		leg4_newXYZ = np.matmul(rot_O_to_leg4, self.XYZ_home_OL) + self.trans_O_to_leg4
		leg5_newXYZ = np.matmul(rot_O_to_leg5, self.XYZ_home_OM) + self.trans_O_to_leg5
		leg6_newXYZ = np.matmul(rot_O_to_leg6, self.XYZ_home_ON) + self.trans_O_to_leg6

		return leg1_newXYZ, leg2_newXYZ, leg3_newXYZ, leg4_newXYZ, leg5_newXYZ, leg6_newXYZ

	def bodyRotate_to_newLegXYZ_customHome(self, r, p, y, new_Z_home):

		## Reverse angle body rotation matrix
		REV_ROT_MUL = self.get_RPY_matrix(r,p,y, -1.0)

		rot_O_to_leg1 = np.matmul(self.leg1_offset_rot, REV_ROT_MUL)
		rot_O_to_leg2 = np.matmul(self.leg2_offset_rot, REV_ROT_MUL)
		rot_O_to_leg3 = np.matmul(self.leg3_offset_rot, REV_ROT_MUL)
		rot_O_to_leg4 = np.matmul(self.leg4_offset_rot, REV_ROT_MUL)
		rot_O_to_leg5 = np.matmul(self.leg5_offset_rot, REV_ROT_MUL)
		rot_O_to_leg6 = np.matmul(self.leg6_offset_rot, REV_ROT_MUL)

		XYZ_home_OI = np.array([(self.X_home+self.C1), 0, new_Z_home])
		XYZ_home_OJ = np.array([(self.X_home+self.C2)*np.cos(self.beta_rad), (self.X_home+self.C2)*np.sin(self.beta_rad), new_Z_home])
		XYZ_home_OK = np.array([-(self.X_home+self.C2)*np.cos(self.beta_rad), (self.X_home+self.C2)*np.sin(self.beta_rad), new_Z_home])
		XYZ_home_OL = np.array([-(self.X_home+self.C1), 0, new_Z_home])
		XYZ_home_OM = np.array([-(self.X_home+self.C2)*np.cos(self.beta_rad), -(self.X_home+self.C2)*np.sin(self.beta_rad), new_Z_home])
		XYZ_home_ON = np.array([(self.X_home+self.C2)*np.cos(self.beta_rad), -(self.X_home+self.C2)*np.sin(self.beta_rad), new_Z_home])

		leg1_newXYZ = np.matmul(rot_O_to_leg1, XYZ_home_OI) + self.trans_O_to_leg1
		leg2_newXYZ = np.matmul(rot_O_to_leg2, XYZ_home_OJ) + self.trans_O_to_leg2
		leg3_newXYZ = np.matmul(rot_O_to_leg3, XYZ_home_OK) + self.trans_O_to_leg3
		leg4_newXYZ = np.matmul(rot_O_to_leg4, XYZ_home_OL) + self.trans_O_to_leg4
		leg5_newXYZ = np.matmul(rot_O_to_leg5, XYZ_home_OM) + self.trans_O_to_leg5
		leg6_newXYZ = np.matmul(rot_O_to_leg6, XYZ_home_ON) + self.trans_O_to_leg6

		return leg1_newXYZ, leg2_newXYZ, leg3_newXYZ, leg4_newXYZ, leg5_newXYZ, leg6_newXYZ

	def generate_crabWalkingLUT(self):
		"""
		Generate Look-Up-Table for Tri-Gait crab-walking motion
		Crab-walking means the robot will not change orientation (heading) of itself
		but it can move side-by-side, diagonal, forward-backward

		Parameters:
			no passing arguments, but the table's size is decided from cw_ang_resolution

		Returns:
			not returns anything, but crab_walking_LUT_THETA will be generated
			crab_walking_LUT_THETA shape is ANGLE RESOLUTION x 18 (no. of joints) x DATA_POINTS

		"""

		for i, dir_rot_ang in enumerate(range(0, 360, self.cw_ang_resolution)):

			XYZ_1 = self.XYZ_TriGait_gen(1, dir_rot_ang)
			XYZ_2 = self.XYZ_TriGait_gen(2, dir_rot_ang)
			XYZ_3 = self.XYZ_TriGait_gen(3, dir_rot_ang)
			XYZ_4 = self.XYZ_TriGait_gen(4, dir_rot_ang)
			XYZ_5 = self.XYZ_TriGait_gen(5, dir_rot_ang)
			XYZ_6 = self.XYZ_TriGait_gen(6, dir_rot_ang)

			THETA1_1, THETA2_1, THETA3_1 = self.invKinArray_to_ThetaArray(np.transpose(XYZ_1))
			THETA1_2, THETA2_2, THETA3_2 = self.invKinArray_to_ThetaArray(np.transpose(XYZ_2))
			THETA1_3, THETA2_3, THETA3_3 = self.invKinArray_to_ThetaArray(np.transpose(XYZ_3))
			THETA1_4, THETA2_4, THETA3_4 = self.invKinArray_to_ThetaArray(np.transpose(XYZ_4))
			THETA1_5, THETA2_5, THETA3_5 = self.invKinArray_to_ThetaArray(np.transpose(XYZ_5))
			THETA1_6, THETA2_6, THETA3_6 = self.invKinArray_to_ThetaArray(np.transpose(XYZ_6))

			lut_theta_elem = np.asarray([THETA1_1, THETA2_1, THETA3_1, 
									THETA1_2, THETA2_2, THETA3_2, 
									THETA1_3, THETA2_3, THETA3_3, 
									THETA1_4, THETA2_4, THETA3_4, 
									THETA1_5, THETA2_5, THETA3_5, 
									THETA1_6, THETA2_6, THETA3_6])

			self.crab_walking_LUT_THETA[i] = np.copy(lut_theta_elem)

	def generate_crabWalkingLUT_custom(self, body_pitch, leg_XYZ):
		"""
		Generate Look-Up-Table for Tri-Gait crab-walking motion but with different home position, it has height or pitch commands

		Parameters:
			no passing arguments, but the table's size is decided from cw_ang_resolution

		Returns:
			not returns anything, but crab_walking_LUT_custom_THETA will be generated
			crab_walking_LUT_custom_THETA shape is ANGLE RESOLUTION x 18 (no. of joints) x DATA_POINTS

		"""

		for i, dir_rot_ang in enumerate(range(0, 360, self.cw_ang_resolution)):
			XYZ_1 = self.XYZ_TriGait_gen_custom(1, dir_rot_ang, body_pitch, leg_XYZ[0][0], leg_XYZ[0][2])
			XYZ_2 = self.XYZ_TriGait_gen_custom(2, dir_rot_ang, body_pitch, leg_XYZ[1][0], leg_XYZ[1][2])
			XYZ_3 = self.XYZ_TriGait_gen_custom(3, dir_rot_ang, body_pitch, leg_XYZ[2][0], leg_XYZ[2][2])
			XYZ_4 = self.XYZ_TriGait_gen_custom(4, dir_rot_ang, body_pitch, leg_XYZ[3][0], leg_XYZ[3][2])
			XYZ_5 = self.XYZ_TriGait_gen_custom(5, dir_rot_ang, body_pitch, leg_XYZ[4][0], leg_XYZ[4][2])
			XYZ_6 = self.XYZ_TriGait_gen_custom(6, dir_rot_ang, body_pitch, leg_XYZ[5][0], leg_XYZ[5][2])

			THETA1_1, THETA2_1, THETA3_1 = self.invKinArray_to_ThetaArray(np.transpose(XYZ_1))
			THETA1_2, THETA2_2, THETA3_2 = self.invKinArray_to_ThetaArray(np.transpose(XYZ_2))
			THETA1_3, THETA2_3, THETA3_3 = self.invKinArray_to_ThetaArray(np.transpose(XYZ_3))
			THETA1_4, THETA2_4, THETA3_4 = self.invKinArray_to_ThetaArray(np.transpose(XYZ_4))
			THETA1_5, THETA2_5, THETA3_5 = self.invKinArray_to_ThetaArray(np.transpose(XYZ_5))
			THETA1_6, THETA2_6, THETA3_6 = self.invKinArray_to_ThetaArray(np.transpose(XYZ_6))

			lut_theta_elem = np.asarray([THETA1_1, THETA2_1, THETA3_1, 
									THETA1_2, THETA2_2, THETA3_2, 
									THETA1_3, THETA2_3, THETA3_3, 
									THETA1_4, THETA2_4, THETA3_4, 
									THETA1_5, THETA2_5, THETA3_5, 
									THETA1_6, THETA2_6, THETA3_6])


			self.crab_walking_LUT_custom_THETA[i] = np.copy(lut_theta_elem)



	def generate_inplace_turning(self, turn_deg=-20):
		"""
		Generate how much of xyz for in-place turning
		leg I is 1 
		leg J is 2 
		leg K is 3 
		leg L is 4 
		leg M is 5 
		leg N is 6 
		then legs I-K-M will have the same motion, also legs J-L-N will have the same motion

		Parameters:
			turn_deg: how much angle to turn in one step, as default 20 degree

		Returns:
			legIKM_XYZ: a set of point for leg I-K-M [[x1,y1,z1], [x2,y2,z2], [x3,y3,z3], ..., [xN, yN, zN] ]
			legJLN_XYZ: a set of point for leg J-L-N [[x1,y1,z1], [x2,y2,z2], [x3,y3,z3], ..., [xN, yN, zN] ]
		"""

		## from lift-down last then drag as theta1 swing angle
		theta1_swing = np.radians(turn_deg)
		theta1_drag = np.linspace(0, theta1_swing, self.data_point_TURN_per_blk)
		
		for i, (the1, the2, the3) in enumerate(zip(theta1_drag, self.theta2_drag, self.theta3_drag)):

			x_drag_pt, y_drag_pt, z_drag_pt = self.fwd(the1, the2, the3)
			xyz_drag_array = np.array([[x_drag_pt, y_drag_pt, z_drag_pt]])

			self.xyz_drag_move[i] = xyz_drag_array

		## from drag last keeps holding that
		xyz_drag_last_array = np.array([self.xyz_drag_move[-1][0], self.xyz_drag_move[-1][1], self.xyz_drag_move[-1][2]])
		self.xyz_drag_hold = np.linspace(xyz_drag_last_array, xyz_drag_last_array, self.data_point_TURN_per_blk)

		## from drag hold then lift-up, this is lift-up moves
		self.xyz_LU_move = np.linspace(xyz_drag_last_array, self.xyz_LU_last, self.data_point_TURN_per_blk)
		

		## assign motion according to legs
		legIKM_blk1 = np.copy(self.xyz_LU_hold)
		legIKM_blk2 = np.copy(self.xyz_LD_move)
		legIKM_blk3 = np.copy(self.xyz_LD_hold)
		legIKM_blk4 = np.copy(self.xyz_drag_move)
		legIKM_blk5 = np.copy(self.xyz_drag_hold)
		legIKM_blk6 = np.copy(self.xyz_LU_move)

		legJLN_blk1 = np.copy(self.xyz_drag_move)
		legJLN_blk2 = np.copy(self.xyz_drag_hold)
		legJLN_blk3 = np.copy(self.xyz_LU_move)
		legJLN_blk4 = np.copy(self.xyz_LU_hold)
		legJLN_blk5 = np.copy(self.xyz_LD_move)
		legJLN_blk6 = np.copy(self.xyz_LD_hold)

		legIKM_XYZ = np.concatenate((legIKM_blk1, legIKM_blk2, legIKM_blk3, legIKM_blk4, legIKM_blk5, legIKM_blk6), axis=0)
		legJLN_XYZ = np.concatenate((legJLN_blk1, legJLN_blk2, legJLN_blk3, legJLN_blk4, legJLN_blk5, legJLN_blk6), axis=0)


		# leg1_theta1, leg1_theta2, leg1_theta3 = self.invKinArray_to_ThetaArray(legIKM_XYZ)
		# leg2_theta1, leg2_theta2, leg2_theta3 = self.invKinArray_to_ThetaArray(legJLN_XYZ)

		return legIKM_XYZ, legJLN_XYZ

	def generate_inplace_turning_customHeight(self, turn_deg, XYZ):
		"""
		Generate how much of xyz for in-place turning, but with custom start height.
		leg I is 1 
		leg J is 2 
		leg K is 3 
		leg L is 4 
		leg M is 5 
		leg N is 6 
		then legs I-K-M will have the same motion, also legs J-L-N will have the same motion

		Parameters:
			turn_deg: how much angle to turn in one step, as default 20 degree

		Returns:
			legIKM_XYZ: a set of point for leg I-K-M [[x1,y1,z1], [x2,y2,z2], [x3,y3,z3], ..., [xN, yN, zN] ]
			legJLN_XYZ: a set of point for leg J-L-N [[x1,y1,z1], [x2,y2,z2], [x3,y3,z3], ..., [xN, yN, zN] ]
		"""
		x_LU_last = XYZ[0]
		y_LU_last = XYZ[1]
		z_LU_last = XYZ[2]+50
		xyz_LU_last = np.array([x_LU_last, y_LU_last, z_LU_last])

		x_LD_last = XYZ[0]
		y_LD_last = XYZ[1]
		z_LD_last = XYZ[2]
		xyz_LD_last = np.array([x_LD_last, y_LD_last, z_LD_last])

		## from lift-up last and keep holding
		xyz_LU_hold = np.linspace(xyz_LU_last, xyz_LU_last, self.data_point_TURN_per_blk)
		## from lift-up hold to lift-down, this is lift-down moves
		xyz_LD_move = np.linspace(xyz_LU_last, xyz_LD_last, self.data_point_TURN_per_blk)
		## from lift-down last keep holding
		xyz_LD_hold = np.linspace(xyz_LD_last, xyz_LD_last, self.data_point_TURN_per_blk)

		theta1_home_custom, theta2_home_custom, theta3_home_custom = self.inv(XYZ[0], XYZ[1], XYZ[2])

		theta2_drag = np.linspace(theta2_home_custom, theta2_home_custom, self.data_point_TURN_per_blk)
		theta3_drag = np.linspace(theta3_home_custom, theta3_home_custom, self.data_point_TURN_per_blk)

		xyz_drag_move = np.empty((self.data_point_TURN_per_blk, 3), dtype=np.float32)

		## from lift-down last then drag as theta1 swing angle
		theta1_swing = np.radians(turn_deg)
		theta1_drag = np.linspace(0, theta1_swing, self.data_point_TURN_per_blk)
		
		for i, (the1, the2, the3) in enumerate(zip(theta1_drag, theta2_drag, theta3_drag)):

			x_drag_pt, y_drag_pt, z_drag_pt = self.fwd(the1, the2, the3)
			xyz_drag_array = np.array([[x_drag_pt, y_drag_pt, z_drag_pt]])

			xyz_drag_move[i] = xyz_drag_array

		## from drag last keeps holding that
		xyz_drag_last_array = np.array([xyz_drag_move[-1][0], xyz_drag_move[-1][1], xyz_drag_move[-1][2]])
		xyz_drag_hold = np.linspace(xyz_drag_last_array, xyz_drag_last_array, self.data_point_TURN_per_blk)

		## from drag hold then lift-up, this is lift-up moves
		xyz_LU_move = np.linspace(xyz_drag_last_array, xyz_LU_last, self.data_point_TURN_per_blk)
		

		## assign motion according to legs
		legIKM_blk1 = np.copy(xyz_LU_hold)
		legIKM_blk2 = np.copy(xyz_LD_move)
		legIKM_blk3 = np.copy(xyz_LD_hold)
		legIKM_blk4 = np.copy(xyz_drag_move)
		legIKM_blk5 = np.copy(xyz_drag_hold)
		legIKM_blk6 = np.copy(xyz_LU_move)

		legJLN_blk1 = np.copy(xyz_drag_move)
		legJLN_blk2 = np.copy(xyz_drag_hold)
		legJLN_blk3 = np.copy(xyz_LU_move)
		legJLN_blk4 = np.copy(xyz_LU_hold)
		legJLN_blk5 = np.copy(xyz_LD_move)
		legJLN_blk6 = np.copy(xyz_LD_hold)

		legIKM_XYZ = np.concatenate((legIKM_blk1, legIKM_blk2, legIKM_blk3, legIKM_blk4, legIKM_blk5, legIKM_blk6), axis=0)
		legJLN_XYZ = np.concatenate((legJLN_blk1, legJLN_blk2, legJLN_blk3, legJLN_blk4, legJLN_blk5, legJLN_blk6), axis=0)


		# leg1_theta1, leg1_theta2, leg1_theta3 = self.invKinArray_to_ThetaArray(legIKM_XYZ)
		# leg2_theta1, leg2_theta2, leg2_theta3 = self.invKinArray_to_ThetaArray(legJLN_XYZ)

		return legIKM_XYZ, legJLN_XYZ

	def XYZ_WaveGait_gen(self, leg_id, dir_rot_ang):
		"""
		Generate XYZ set of wave-gait motion of input leg_id, this will be used in generate_waveGaitLUT()

		Parameters:
			leg_id: leg's number 1, 2, 3, 4, 5, or 6
			dir_rot_ang: direction angle of robot's facing similar to Tri-Gait

		Returns:
			XYZ_R[0]: array of X values
			XYZ_R[1]: array of Y values
			XYZ_R[2]: array of Z values
		"""

		S = self.Z_home
		T = 100.0 #200
		A = 100 #80

		P1 = [-T/2, S]
		P2 = [0, (S+(2*A))]
		P3 = [T/2, S]


		t = np.linspace(0,1, self.wg_data_per_blk)
		x_start = self.x_start

		line_data_blk = int(self.wg_data_per_blk*5)

		full_Y_line = np.linspace(T/2, -T/2, line_data_blk)
		full_Z_line = np.linspace(S, S, line_data_blk)
		full_X_line = np.linspace(0.0, 0.0, line_data_blk)

		X_line5 = full_X_line[:self.wg_data_per_blk]
		Y_line5 = full_Y_line[:self.wg_data_per_blk]
		Z_line5 = full_Z_line[:self.wg_data_per_blk]

		X_line4 = full_X_line[self.wg_data_per_blk:int(2*self.wg_data_per_blk)]
		Y_line4 = full_Y_line[self.wg_data_per_blk:int(2*self.wg_data_per_blk)]
		Z_line4 = full_Z_line[self.wg_data_per_blk:int(2*self.wg_data_per_blk)]

		X_line3 = full_X_line[int(2*self.wg_data_per_blk):int(3*self.wg_data_per_blk)]
		Y_line3 = full_Y_line[int(2*self.wg_data_per_blk):int(3*self.wg_data_per_blk)]
		Z_line3 = full_Z_line[int(2*self.wg_data_per_blk):int(3*self.wg_data_per_blk)]

		X_line2 = full_X_line[int(3*self.wg_data_per_blk):int(4*self.wg_data_per_blk)]
		Y_line2 = full_Y_line[int(3*self.wg_data_per_blk):int(4*self.wg_data_per_blk)]
		Z_line2 = full_Z_line[int(3*self.wg_data_per_blk):int(4*self.wg_data_per_blk)]

		X_line1 = full_X_line[int(4*self.wg_data_per_blk):]
		Y_line1 = full_Y_line[int(4*self.wg_data_per_blk):]
		Z_line1 = full_Z_line[int(4*self.wg_data_per_blk):]


		Y_curve = (((1-t)**2)*P1[0]) + 2*(1-t)*t*P2[0] + (t**2)*P3[0]
		Z_curve = (((1-t)**2)*P1[1]) + 2*(1-t)*t*P2[1] + (t**2)*P3[1]
		X_curve = np.linspace(0.0, 0.0, self.wg_data_per_blk)

		X_line5_flipped = np.flip(X_line5, 0)
		Y_line5_flipped = np.flip(Y_line5, 0)
		Z_line5_flipped = np.flip(Z_line5, 0)

		X_line4_flipped = np.flip(X_line4, 0)
		Y_line4_flipped = np.flip(Y_line4, 0)
		Z_line4_flipped = np.flip(Z_line4, 0)

		X_line3_flipped = np.flip(X_line3, 0)
		Y_line3_flipped = np.flip(Y_line3, 0)
		Z_line3_flipped = np.flip(Z_line3, 0)

		X_line2_flipped = np.flip(X_line2, 0)
		Y_line2_flipped = np.flip(Y_line2, 0)
		Z_line2_flipped = np.flip(Z_line2, 0)

		X_line1_flipped = np.flip(X_line1, 0)
		Y_line1_flipped = np.flip(Y_line1, 0)
		Z_line1_flipped = np.flip(Z_line1, 0)


		X_curve_flipped = np.flip(X_curve, 0)
		Y_curve_flipped = np.flip(Y_curve, 0)
		Z_curve_flipped = np.flip(Z_curve, 0)

		if leg_id == 1:
			rot = self.walk_rot_ang_1 + np.radians(dir_rot_ang)
		elif leg_id == 2:
			rot = self.walk_rot_ang_2 + np.radians(dir_rot_ang)
		elif leg_id == 3:
			rot = self.walk_rot_ang_3 + np.radians(dir_rot_ang)
		elif leg_id == 4:
			rot = self.walk_rot_ang_4 + np.radians(dir_rot_ang)
		elif leg_id == 5:
			rot = self.walk_rot_ang_5 + np.radians(dir_rot_ang)
		elif leg_id == 6:
			rot = self.walk_rot_ang_6 + np.radians(dir_rot_ang)


		if (leg_id == 1):

			X = np.concatenate((X_curve, X_line5, X_line4, X_line3, X_line2, X_line1), axis=0)
			Y = np.concatenate((Y_curve, Y_line5, Y_line4, Y_line3, Y_line2, Y_line1), axis=0)
			Z = np.concatenate((Z_curve, Z_line5, Z_line4, Z_line3, Z_line2, Z_line1), axis=0)


		elif (leg_id == 2):

			X = np.concatenate((X_line1, X_curve, X_line5, X_line4, X_line3, X_line2), axis=0)
			Y = np.concatenate((Y_line1, Y_curve, Y_line5, Y_line4, Y_line3, Y_line2), axis=0)
			Z = np.concatenate((Z_line1, Z_curve, Z_line5, Z_line4, Z_line3, Z_line2), axis=0)


		elif (leg_id == 3):

			X = np.concatenate((X_line4_flipped, X_line5_flipped, X_curve_flipped, X_line1_flipped, X_line2_flipped, X_line3_flipped), axis=0)
			Y = np.concatenate((Y_line4_flipped, Y_line5_flipped, Y_curve_flipped, Y_line1_flipped, Y_line2_flipped, Y_line3_flipped), axis=0)
			Z = np.concatenate((Z_line4_flipped, Z_line5_flipped, Z_curve_flipped, Z_line1_flipped, Z_line2_flipped, Z_line3_flipped), axis=0)

		elif (leg_id == 4):

			# X = np.concatenate((X_line3_flipped, X_line4_flipped, X_line5_flipped, X_curve_flipped, X_line1_flipped, X_line2_flipped), axis=0)
			# Y = np.concatenate((Y_line3_flipped, Y_line4_flipped, Y_line5_flipped, Y_curve_flipped, Y_line1_flipped, Y_line2_flipped), axis=0)
			# Z = np.concatenate((Z_line3_flipped, Z_line4_flipped, Z_line5_flipped, Z_curve_flipped, Z_line1_flipped, Z_line2_flipped), axis=0)

			X = np.concatenate((X_line3, X_line2, X_line1, X_curve, X_line5, X_line4), axis=0)
			Y = np.concatenate((Y_line3, Y_line2, Y_line1, Y_curve, Y_line5, Y_line4), axis=0)
			Z = np.concatenate((Z_line3, Z_line2, Z_line1, Z_curve, Z_line5, Z_line4), axis=0)


		elif (leg_id == 5):
			X = np.concatenate((X_line2_flipped, X_line3_flipped, X_line4_flipped, X_line5_flipped, X_curve_flipped, X_line1_flipped), axis=0)
			Y = np.concatenate((Y_line2_flipped, Y_line3_flipped, Y_line4_flipped, Y_line5_flipped, Y_curve_flipped, Y_line1_flipped), axis=0)
			Z = np.concatenate((Z_line2_flipped, Z_line3_flipped, Z_line4_flipped, Z_line5_flipped, Z_curve_flipped, Z_line1_flipped), axis=0)


		elif (leg_id == 6):
			X = np.concatenate((X_line5, X_line4, X_line3, X_line2, X_line1, X_curve), axis=0)
			Y = np.concatenate((Y_line5, Y_line4, Y_line3, Y_line2, Y_line1, Y_curve), axis=0)
			Z = np.concatenate((Z_line5, Z_line4, Z_line3, Z_line2, Z_line1, Z_curve), axis=0)


		### rotation matrix of rot matrix ###
		ROT = np.array([[np.cos(rot), -np.sin(rot), 0], 
						[np.sin(rot), np.cos(rot), 0], 
						[0, 0, 1]])

		XYZ = np.stack((X,Y,Z), axis=-1)

		### transform XYZ of (0,0,0) frame to each leg frame ###
		XYZ_R = np.array([])
		for i in range(len(XYZ)):

			xyz_r = np.matmul(ROT, XYZ[i].T)
			xyz_r = np.expand_dims(xyz_r, axis=0)
			if i == 0:
				XYZ_R = np.append(XYZ_R, xyz_r)
				XYZ_R = np.expand_dims(XYZ_R, axis=0)
			else:
				XYZ_R = np.concatenate((XYZ_R, xyz_r))

		XYZ_R[:,0] = XYZ_R[:,0] + self.x_start

		XYZ_R = np.transpose(XYZ_R)

		return XYZ_R[0], XYZ_R[1], XYZ_R[2]

	def XYZ_WaveGait_realTime(self, leg_id, ang_index, turn_ratio, home_xyz):
		"""
		Generate XYZ set of wave-gait motion in realtime

		Parameters:
			leg_id: leg's number 1, 2, 3, 4, 5, or 6
			ang_index: the index angle of crab walking direction, value is from 0 -> 35
			turn_ratio: the percentage to move on either left or right sides longer
			home_xyz: starting position

		Returns:
			XYZ_R[0]: array of X values
			XYZ_R[1]: array of Y values
			XYZ_R[2]: array of Z values
		"""

		# dir_rot_ang = 0
		dir_rot_ang = ang_index*self.wg_ang_resolution ## ang_index comes as 0,1,2,3,...,35

		S = 0.0 #self.Z_home
		T = 150.0 + (40.0*turn_ratio) #200
		A = 150 #80

		P1 = [-T/2, S]
		P2 = [0, (S+(2*A))]
		P3 = [T/2, S]


		t = np.linspace(0,1, self.wg_data_per_blk)
		x_start = home_xyz[0] #self.x_start
		z_start = home_xyz[2]

		line_data_blk = int(self.wg_data_per_blk*5)

		full_Y_line = np.linspace(T/2, -T/2, line_data_blk)
		full_Z_line = np.linspace(S, S, line_data_blk)
		full_X_line = np.linspace(0.0, 0.0, line_data_blk)

		X_line5 = full_X_line[:self.wg_data_per_blk]
		Y_line5 = full_Y_line[:self.wg_data_per_blk]
		Z_line5 = full_Z_line[:self.wg_data_per_blk]

		X_line4 = full_X_line[self.wg_data_per_blk:int(2*self.wg_data_per_blk)]
		Y_line4 = full_Y_line[self.wg_data_per_blk:int(2*self.wg_data_per_blk)]
		Z_line4 = full_Z_line[self.wg_data_per_blk:int(2*self.wg_data_per_blk)]

		X_line3 = full_X_line[int(2*self.wg_data_per_blk):int(3*self.wg_data_per_blk)]
		Y_line3 = full_Y_line[int(2*self.wg_data_per_blk):int(3*self.wg_data_per_blk)]
		Z_line3 = full_Z_line[int(2*self.wg_data_per_blk):int(3*self.wg_data_per_blk)]

		X_line2 = full_X_line[int(3*self.wg_data_per_blk):int(4*self.wg_data_per_blk)]
		Y_line2 = full_Y_line[int(3*self.wg_data_per_blk):int(4*self.wg_data_per_blk)]
		Z_line2 = full_Z_line[int(3*self.wg_data_per_blk):int(4*self.wg_data_per_blk)]

		X_line1 = full_X_line[int(4*self.wg_data_per_blk):]
		Y_line1 = full_Y_line[int(4*self.wg_data_per_blk):]
		Z_line1 = full_Z_line[int(4*self.wg_data_per_blk):]


		Y_curve = (((1-t)**2)*P1[0]) + 2*(1-t)*t*P2[0] + (t**2)*P3[0]
		Z_curve = (((1-t)**2)*P1[1]) + 2*(1-t)*t*P2[1] + (t**2)*P3[1]
		X_curve = np.linspace(0.0, 0.0, self.wg_data_per_blk)

		X_line5_flipped = np.flip(X_line5, 0)
		Y_line5_flipped = np.flip(Y_line5, 0)
		Z_line5_flipped = np.flip(Z_line5, 0)

		X_line4_flipped = np.flip(X_line4, 0)
		Y_line4_flipped = np.flip(Y_line4, 0)
		Z_line4_flipped = np.flip(Z_line4, 0)

		X_line3_flipped = np.flip(X_line3, 0)
		Y_line3_flipped = np.flip(Y_line3, 0)
		Z_line3_flipped = np.flip(Z_line3, 0)

		X_line2_flipped = np.flip(X_line2, 0)
		Y_line2_flipped = np.flip(Y_line2, 0)
		Z_line2_flipped = np.flip(Z_line2, 0)

		X_line1_flipped = np.flip(X_line1, 0)
		Y_line1_flipped = np.flip(Y_line1, 0)
		Z_line1_flipped = np.flip(Z_line1, 0)


		X_curve_flipped = np.flip(X_curve, 0)
		Y_curve_flipped = np.flip(Y_curve, 0)
		Z_curve_flipped = np.flip(Z_curve, 0)

		if leg_id == 1:
			rot = self.walk_rot_ang_1 + np.radians(dir_rot_ang)
		elif leg_id == 2:
			rot = self.walk_rot_ang_2 + np.radians(dir_rot_ang)
		elif leg_id == 3:
			rot = self.walk_rot_ang_3 + np.radians(dir_rot_ang)
		elif leg_id == 4:
			rot = self.walk_rot_ang_4 + np.radians(dir_rot_ang)
		elif leg_id == 5:
			rot = self.walk_rot_ang_5 + np.radians(dir_rot_ang)
		elif leg_id == 6:
			rot = self.walk_rot_ang_6 + np.radians(dir_rot_ang)


		if (leg_id == 1):

			X = np.concatenate((X_curve, X_line5, X_line4, X_line3, X_line2, X_line1), axis=0)
			Y = np.concatenate((Y_curve, Y_line5, Y_line4, Y_line3, Y_line2, Y_line1), axis=0)
			Z = np.concatenate((Z_curve, Z_line5, Z_line4, Z_line3, Z_line2, Z_line1), axis=0)


		elif (leg_id == 2):

			X = np.concatenate((X_line1, X_curve, X_line5, X_line4, X_line3, X_line2), axis=0)
			Y = np.concatenate((Y_line1, Y_curve, Y_line5, Y_line4, Y_line3, Y_line2), axis=0)
			Z = np.concatenate((Z_line1, Z_curve, Z_line5, Z_line4, Z_line3, Z_line2), axis=0)


		elif (leg_id == 3):

			X = np.concatenate((X_line4_flipped, X_line5_flipped, X_curve_flipped, X_line1_flipped, X_line2_flipped, X_line3_flipped), axis=0)
			Y = np.concatenate((Y_line4_flipped, Y_line5_flipped, Y_curve_flipped, Y_line1_flipped, Y_line2_flipped, Y_line3_flipped), axis=0)
			Z = np.concatenate((Z_line4_flipped, Z_line5_flipped, Z_curve_flipped, Z_line1_flipped, Z_line2_flipped, Z_line3_flipped), axis=0)

		elif (leg_id == 4):

			# X = np.concatenate((X_line3_flipped, X_line4_flipped, X_line5_flipped, X_curve_flipped, X_line1_flipped, X_line2_flipped), axis=0)
			# Y = np.concatenate((Y_line3_flipped, Y_line4_flipped, Y_line5_flipped, Y_curve_flipped, Y_line1_flipped, Y_line2_flipped), axis=0)
			# Z = np.concatenate((Z_line3_flipped, Z_line4_flipped, Z_line5_flipped, Z_curve_flipped, Z_line1_flipped, Z_line2_flipped), axis=0)

			X = np.concatenate((X_line3, X_line2, X_line1, X_curve, X_line5, X_line4), axis=0)
			Y = np.concatenate((Y_line3, Y_line2, Y_line1, Y_curve, Y_line5, Y_line4), axis=0)
			Z = np.concatenate((Z_line3, Z_line2, Z_line1, Z_curve, Z_line5, Z_line4), axis=0)


		elif (leg_id == 5):
			X = np.concatenate((X_line2_flipped, X_line3_flipped, X_line4_flipped, X_line5_flipped, X_curve_flipped, X_line1_flipped), axis=0)
			Y = np.concatenate((Y_line2_flipped, Y_line3_flipped, Y_line4_flipped, Y_line5_flipped, Y_curve_flipped, Y_line1_flipped), axis=0)
			Z = np.concatenate((Z_line2_flipped, Z_line3_flipped, Z_line4_flipped, Z_line5_flipped, Z_curve_flipped, Z_line1_flipped), axis=0)


		elif (leg_id == 6):
			X = np.concatenate((X_line5, X_line4, X_line3, X_line2, X_line1, X_curve), axis=0)
			Y = np.concatenate((Y_line5, Y_line4, Y_line3, Y_line2, Y_line1, Y_curve), axis=0)
			Z = np.concatenate((Z_line5, Z_line4, Z_line3, Z_line2, Z_line1, Z_curve), axis=0)


		### rotation matrix of rot matrix ###
		ROT = np.array([[np.cos(rot), -np.sin(rot), 0], 
						[np.sin(rot), np.cos(rot), 0], 
						[0, 0, 1]])

		XYZ = np.stack((X,Y,Z), axis=-1)

		### transform XYZ of (0,0,0) frame to each leg frame ###
		XYZ_R = np.array([])
		for i in range(len(XYZ)):

			xyz_r = np.matmul(ROT, XYZ[i].T)
			xyz_r = np.expand_dims(xyz_r, axis=0)
			if i == 0:
				XYZ_R = np.append(XYZ_R, xyz_r)
				XYZ_R = np.expand_dims(XYZ_R, axis=0)
			else:
				XYZ_R = np.concatenate((XYZ_R, xyz_r))

		# XYZ_R[:,0] = XYZ_R[:,0] + self.x_start
		## shift X
		XYZ_R[:,0] = XYZ_R[:,0] + x_start
		## shift Z
		XYZ_R[:,2] = XYZ_R[:,2] + z_start

		XYZ_R = np.transpose(XYZ_R)

		return XYZ_R[0], XYZ_R[1], XYZ_R[2]

	def generate_waveGaitLUT(self):
		"""
		Generate Look-Up-Table for Wave-Gait crab-walking motion
		Crab-walking means the robot will not change orientation (heading) of itself
		but it can move side-by-side, diagonal, forward-backward

		Parameters:
			no passing arguments, but the table's size is decided from cw_ang_resolution

		Returns:
			not returns anything, but wave_gait_LUT_THETA will be generated
			wave_gait_LUT_THETA shape is ANGLE RESOLUTION x 18 (no. of joints) x DATA_POINTS

		"""

		for i, dir_rot_ang in enumerate(range(0, 360, self.wg_ang_resolution)):

			XYZ_1 = self.XYZ_WaveGait_gen(1, dir_rot_ang)
			XYZ_2 = self.XYZ_WaveGait_gen(2, dir_rot_ang)
			XYZ_3 = self.XYZ_WaveGait_gen(3, dir_rot_ang)
			XYZ_4 = self.XYZ_WaveGait_gen(4, dir_rot_ang)
			XYZ_5 = self.XYZ_WaveGait_gen(5, dir_rot_ang)
			XYZ_6 = self.XYZ_WaveGait_gen(6, dir_rot_ang)

			THETA1_1, THETA2_1, THETA3_1 = self.invKinArray_to_ThetaArray(np.transpose(XYZ_1))
			THETA1_2, THETA2_2, THETA3_2 = self.invKinArray_to_ThetaArray(np.transpose(XYZ_2))
			THETA1_3, THETA2_3, THETA3_3 = self.invKinArray_to_ThetaArray(np.transpose(XYZ_3))
			THETA1_4, THETA2_4, THETA3_4 = self.invKinArray_to_ThetaArray(np.transpose(XYZ_4))
			THETA1_5, THETA2_5, THETA3_5 = self.invKinArray_to_ThetaArray(np.transpose(XYZ_5))
			THETA1_6, THETA2_6, THETA3_6 = self.invKinArray_to_ThetaArray(np.transpose(XYZ_6))

			lut_theta_elem = np.asarray([THETA1_1, THETA2_1, THETA3_1, 
									THETA1_2, THETA2_2, THETA3_2, 
									THETA1_3, THETA2_3, THETA3_3, 
									THETA1_4, THETA2_4, THETA3_4, 
									THETA1_5, THETA2_5, THETA3_5, 
									THETA1_6, THETA2_6, THETA3_6])

			self.wave_gait_LUT_THETA[i] = np.copy(lut_theta_elem)
		



if __name__ == "__main__":

	h = SpiderBotLib()
	start_time = time.time()
	h.generate_crabWalkingLUT()
	period = time.time() - start_time
	# print(period)

	# r = np.radians(15.0)
	# p = np.radians(0.0)
	# y = np.radians(0.0)
	# leg_XYZ  = h.bodyRotate_to_newLegXYZ(r,p,y)
	# print("leg_XYZ[0]", leg_XYZ[0])

	# dir_rot_ang = 0
	# XYZ_1 = h.XYZ_gen_custom(1, dir_rot_ang, r, leg_XYZ[0][0], leg_XYZ[0][2])
	# XYZ_2 = h.XYZ_gen_custom(2, dir_rot_ang, r, leg_XYZ[1][0], leg_XYZ[1][2])
	# XYZ_3 = h.XYZ_gen_custom(3, dir_rot_ang, r, leg_XYZ[2][0], leg_XYZ[2][2])
	# XYZ_4 = h.XYZ_gen_custom(4, dir_rot_ang, r, leg_XYZ[3][0], leg_XYZ[3][2])
	# XYZ_5 = h.XYZ_gen_custom(5, dir_rot_ang, r, leg_XYZ[4][0], leg_XYZ[4][2])
	# XYZ_6 = h.XYZ_gen_custom(6, dir_rot_ang, r, leg_XYZ[5][0], leg_XYZ[5][2])

	# THETA1_1, THETA2_1, THETA3_1 = h.invKinArray_to_ThetaArray(np.transpose(XYZ_1))
	# THETA1_2, THETA2_2, THETA3_2 = h.invKinArray_to_ThetaArray(np.transpose(XYZ_2))
	# THETA1_3, THETA2_3, THETA3_3 = h.invKinArray_to_ThetaArray(np.transpose(XYZ_3))
	# THETA1_4, THETA2_4, THETA3_4 = h.invKinArray_to_ThetaArray(np.transpose(XYZ_4))
	# THETA1_5, THETA2_5, THETA3_5 = h.invKinArray_to_ThetaArray(np.transpose(XYZ_5))
	# THETA1_6, THETA2_6, THETA3_6 = h.invKinArray_to_ThetaArray(np.transpose(XYZ_6))

	# print("XYZ_1", XYZ_1)

	# h.generate_waveGaitLUT()

	# start_time = time.time()
	# XYZ_1 = h.XYZ_WaveGait_gen(1, 0)
	# wg_period  = time.time() - start_time
	# start_time2 = time.time()
	# THETA1_1, THETA2_1, THETA3_1 = h.invKinArray_to_ThetaArray(np.transpose(XYZ_1))
	# inv_period = time.time() - start_time2
	# print(wg_period, inv_period)

	# XYZ_2 = h.XYZ_WaveGait_gen(2, 0)
	# XYZ_3 = h.XYZ_WaveGait_gen(3, 0)
	# XYZ_4 = h.XYZ_WaveGait_gen(4, 0)
	# XYZ_5 = h.XYZ_WaveGait_gen(5, 0)
	# XYZ_6 = h.XYZ_WaveGait_gen(6, 0)

	# fig1 = plt.figure(1)
	# fig1.suptitle('3D', fontsize=10)
	# ax_3d = plt.axes(projection='3d')
	# ax_3d.set_xlabel('x')
	# ax_3d.set_ylabel('y')
	# ax_3d.set_zlabel('z')
	# ax_3d.set_xlim(-300,300)
	# ax_3d.set_ylim(-300,300)
	# ax_3d.set_zlim(-300,100)
	# ax_3d.plot3D(XYZ_1[0], XYZ_1[1], XYZ_1[2], 'red')
	# ax_3d.plot3D(XYZ_2[0], XYZ_2[1], XYZ_2[2], 'green')
	# ax_3d.plot3D(XYZ_3[0], XYZ_3[1], XYZ_3[2], 'blue')
	# ax_3d.plot3D(XYZ_4[0], XYZ_4[1], XYZ_4[2], 'cyan')
	# ax_3d.plot3D(XYZ_5[0], XYZ_5[1], XYZ_5[2], 'magenta')
	# ax_3d.plot3D(XYZ_6[0], XYZ_6[1], XYZ_6[2], 'yellow')
	# plt.show()

	# print("servo1", np.degrees(h.crab_walking_LUT_THETA[0][0]))
	# print("servo2", np.degrees(h.crab_walking_LUT_THETA[0][1]))
	# print("servo3", np.degrees(h.crab_walking_LUT_THETA[0][2]))

	# h.generate_normalWalking_LUT()
	# print("0 0")
	# print(h.normWalking_LUT_PWM[0][0])
	# print("0 1")
	# print(h.normWalking_LUT_PWM[0][1])
	# print("0 2")
	# print(h.normWalking_LUT_PWM[0][2])

	# print("1 0")
	# print(h.normWalking_LUT_PWM[1][0])
	# print("1 1")
	# print(h.normWalking_LUT_PWM[1][1])
	# print("1 2")
	# print(h.normWalking_LUT_PWM[1][2])

	# print("10 0")
	# print(h.normWalking_LUT_PWM[10][0])
	# print("10 4")
	# print(h.normWalking_LUT_PWM[10][4])
	# print("10 10")
	# print(h.normWalking_LUT_PWM[10][10])


	# h.generate_steering_curve(str_sign=-1.0, R_icc=1000)

	# XYZ_1, XYZ_2 = h.generate_inplace_turning(20)
	# XYZ_3 = XYZ_1
	# XYZ_5 = XYZ_1
	# XYZ_4 = XYZ_2
	# XYZ_6 = XYZ_2

	# leg1_CCW_theta1, leg1_CCW_theta2, leg1_CCW_theta3 = h.invKinArray_to_ThetaArray(XYZ_1)
	# leg2_CCW_theta1, leg2_CCW_theta2, leg2_CCW_theta3 = h.invKinArray_to_ThetaArray(XYZ_2)
	# PWM1_CCW_1, PWM2_CCW_1, PWM3_CCW_1 = h.invKinArray_to_PwmArray(np.transpose(XYZ_1)[0],np.transpose(XYZ_1)[1],np.transpose(XYZ_1)[2])
	# PWM1_CCW_2, PWM2_CCW_2, PWM3_CCW_2 = h.invKinArray_to_PwmArray(np.transpose(XYZ_2)[0],np.transpose(XYZ_2)[1],np.transpose(XYZ_2)[2])
	# print("PWM1_CCW_1")
	# print(PWM1_CCW_1)
	# print("PWM2_CCW_1")
	# print(PWM2_CCW_1)
	# print("PWM3_CCW_1")
	# print(PWM3_CCW_1)
	# print(" ")
	# print("PWM1_CCW_2")
	# print(PWM1_CCW_2)
	# print("PWM2_CCW_2")
	# print(PWM2_CCW_2)
	# print("PWM3_CCW_2")
	# print(PWM3_CCW_2)

	# print("leg1_CCW_theta1")
	# print(leg1_CCW_theta1)
	# print("leg1_CCW_theta2")
	# print(leg1_CCW_theta2)
	# print("leg1_CCW_theta3")
	# print(leg1_CCW_theta3)

	# print("leg2_CCW_theta1")
	# print(leg2_CCW_theta1)
	# print("leg2_CCW_theta2")
	# print(leg2_CCW_theta2)
	# print("leg2_CCW_theta3")
	# print(leg2_CCW_theta3)

	# print(" ")

	# xyz_1, xyz_2 = h.generate_inplace_turning(-20)
	# leg1_CW_theta1, leg1_CW_theta2, leg1_CW_theta3 = h.invKinArray_to_ThetaArray(xyz_1)
	# leg2_CW_theta1, leg2_CW_theta2, leg2_CW_theta3 = h.invKinArray_to_ThetaArray(xyz_2)
	# PWM1_CW_1, PWM2_CW_1, PWM3_CW_1 = h.invKinArray_to_PwmArray(np.transpose(xyz_1)[0],np.transpose(xyz_1)[1],np.transpose(xyz_1)[2])
	# PWM1_CW_2, PWM2_CW_2, PWM3_CW_2 = h.invKinArray_to_PwmArray(np.transpose(xyz_2)[0],np.transpose(xyz_2)[1],np.transpose(xyz_2)[2])
	# print("PWM1_CW_1")
	# print(PWM1_CW_1)
	# print("PWM2_CW_1")
	# print(PWM2_CW_1)
	# print("PWM3_CW_1")
	# print(PWM3_CW_1)
	# print(" ")
	# print("PWM1_CW_2")
	# print(PWM1_CW_2)
	# print("PWM2_CW_2")
	# print(PWM2_CW_2)
	# print("PWM3_CW_2")
	# print(PWM3_CW_2)

	# print("leg1_CW_theta1")
	# print(leg1_CW_theta1)
	# print("leg1_CW_theta2")
	# print(leg1_CW_theta2)
	# print("leg1_CW_theta3")
	# print(leg1_CW_theta3)

	# print("leg2_CW_theta1")
	# print(leg2_CW_theta1)
	# print("leg2_CW_theta2")
	# print(leg2_CW_theta2)
	# print("leg2_CW_theta3")
	# print(leg2_CW_theta3)
	# print(" ")


	### Crab-walking ###
	# XYZ_1 = h.XYZ_gen(1, 0)
	# XYZ_2 = h.XYZ_gen(2, 0)
	# XYZ_6 = h.XYZ_gen(6, 0)

	# PWM1_1, PWM2_1, PWM3_1 = h.invKinArray_to_PwmArray(XYZ_1[0],XYZ_1[1],XYZ_1[2])
	# PWM1_2, PWM2_2, PWM3_2 = h.invKinArray_to_PwmArray(XYZ_2[0],XYZ_2[1],XYZ_2[2])
	# PWM1_3, PWM2_3, PWM3_3 = h.invKinArray_to_PwmArray(XYZ_3[0],XYZ_3[1],XYZ_3[2])
	# PWM1_4, PWM2_4, PWM3_4 = h.invKinArray_to_PwmArray(XYZ_4[0],XYZ_4[1],XYZ_4[2])
	# PWM1_5, PWM2_5, PWM3_5 = h.invKinArray_to_PwmArray(XYZ_5[0],XYZ_5[1],XYZ_5[2])
	# PWM1_6, PWM2_6, PWM3_6 = h.invKinArray_to_PwmArray(XYZ_6[0],XYZ_6[1],XYZ_6[2])

	# print("Leg 1 (leg i)")
	# print("X1", XYZ_1[0])
	# print("Y1", XYZ_1[1])
	# print("Z1", XYZ_1[2])
	# print("PWM1_1", PWM1_1)
	# print("PWM2_1", PWM2_1)
	# print("PWM3_1", PWM3_1)

	# print("Leg 2 (leg j)")
	# print("X2", XYZ_2[0])
	# print("Y2", XYZ_2[1])
	# print("Z2", XYZ_2[2])
	# print("PWM1_2", PWM1_2)
	# print("PWM2_2", PWM2_2)
	# print("PWM3_2", PWM3_2)

	# print("Leg 6 (leg n)")
	# print("X6", XYZ_6[0])
	# print("Y6", XYZ_6[1])
	# print("Z6", XYZ_6[2])
	# print("PWM1_6", PWM1_6)
	# print("PWM2_6", PWM2_6)
	# print("PWM3_6", PWM3_6)


	### Body translation & rotation ###
	# PC_new = np.array([0, 0, 40])
	# leg_XYZ  = h.bodyTranslate_to_newLegXYZ(PC_new)

	# r = np.radians(0)
	# p = np.radians(0)
	# y = np.radians(-10)
	# leg_XYZ  = h.bodyRotate_to_newLegXYZ(r,p,y)

	# leg_i_XYZ = leg_XYZ[0]
	# leg_j_XYZ = leg_XYZ[1]
	# leg_k_XYZ = leg_XYZ[2]
	# leg_l_XYZ = leg_XYZ[3]
	# leg_m_XYZ = leg_XYZ[4]
	# leg_n_XYZ = leg_XYZ[5]

	# print("leg_i_XYZ", leg_i_XYZ)
	# print("leg_j_XYZ", leg_j_XYZ)
	# print("leg_k_XYZ", leg_k_XYZ)
	# print("leg_l_XYZ", leg_l_XYZ)
	# print("leg_m_XYZ", leg_m_XYZ)
	# print("leg_n_XYZ", leg_n_XYZ)

	# leg_i_theta = h.inv(leg_i_XYZ[0], leg_i_XYZ[1], leg_i_XYZ[2])
	# leg_j_theta = h.inv(leg_j_XYZ[0], leg_j_XYZ[1], leg_j_XYZ[2])
	# leg_k_theta = h.inv(leg_k_XYZ[0], leg_k_XYZ[1], leg_k_XYZ[2])
	# leg_l_theta = h.inv(leg_l_XYZ[0], leg_l_XYZ[1], leg_l_XYZ[2])
	# leg_m_theta = h.inv(leg_m_XYZ[0], leg_m_XYZ[1], leg_m_XYZ[2])
	# leg_n_theta = h.inv(leg_n_XYZ[0], leg_n_XYZ[1], leg_n_XYZ[2])


	# print("leg_i_theta", leg_i_theta)
	# print("leg_j_theta", leg_j_theta)
	# print("leg_k_theta", leg_k_theta)
	# print("leg_l_theta", leg_l_theta)
	# print("leg_m_theta", leg_m_theta)
	# print("leg_n_theta", leg_n_theta)

	# leg_i_PWM1 = h.kinAgnle_to_servoPWM(0, np.degrees(leg_i_theta[0]))
	# leg_i_PWM2 = h.kinAgnle_to_servoPWM(1, np.degrees(leg_i_theta[1]))
	# leg_i_PWM3 = h.kinAgnle_to_servoPWM(2, np.degrees(leg_i_theta[2]))
	# leg_j_PWM1 = h.kinAgnle_to_servoPWM(0, np.degrees(leg_j_theta[0]))
	# leg_j_PWM2 = h.kinAgnle_to_servoPWM(1, np.degrees(leg_j_theta[1]))
	# leg_j_PWM3 = h.kinAgnle_to_servoPWM(2, np.degrees(leg_j_theta[2]))
	# leg_k_PWM1 = h.kinAgnle_to_servoPWM(0, np.degrees(leg_k_theta[0]))
	# leg_k_PWM2 = h.kinAgnle_to_servoPWM(1, np.degrees(leg_k_theta[1]))
	# leg_k_PWM3 = h.kinAgnle_to_servoPWM(2, np.degrees(leg_k_theta[2]))
	# leg_l_PWM1 = h.kinAgnle_to_servoPWM(0, np.degrees(leg_l_theta[0]))
	# leg_l_PWM2 = h.kinAgnle_to_servoPWM(1, np.degrees(leg_l_theta[1]))
	# leg_l_PWM3 = h.kinAgnle_to_servoPWM(2, np.degrees(leg_l_theta[2]))
	# leg_m_PWM1 = h.kinAgnle_to_servoPWM(0, np.degrees(leg_m_theta[0]))
	# leg_m_PWM2 = h.kinAgnle_to_servoPWM(1, np.degrees(leg_m_theta[1]))
	# leg_m_PWM3 = h.kinAgnle_to_servoPWM(2, np.degrees(leg_m_theta[2]))
	# leg_n_PWM1 = h.kinAgnle_to_servoPWM(0, np.degrees(leg_n_theta[0]))
	# leg_n_PWM2 = h.kinAgnle_to_servoPWM(1, np.degrees(leg_n_theta[1]))
	# leg_n_PWM3 = h.kinAgnle_to_servoPWM(2, np.degrees(leg_n_theta[2]))

	# print("leg_i_PWM", leg_i_PWM1, leg_i_PWM2, leg_i_PWM3)
	# print("leg_j_PWM", leg_j_PWM1, leg_j_PWM2, leg_j_PWM3)
	# print("leg_k_PWM", leg_k_PWM1, leg_k_PWM2, leg_k_PWM3)
	# print("leg_l_PWM", leg_l_PWM1, leg_l_PWM2, leg_l_PWM3)
	# print("leg_m_PWM", leg_m_PWM1, leg_m_PWM2, leg_m_PWM3)
	# print("leg_n_PWM", leg_n_PWM1, leg_n_PWM2, leg_n_PWM3)





	# fig1 = plt.figure(1)
	# fig1.suptitle('3D', fontsize=10)
	# ax_3d = plt.axes(projection='3d')
	# ax_3d.set_xlabel('x')
	# ax_3d.set_ylabel('y')
	# ax_3d.set_zlabel('z')
	# ax_3d.set_xlim(-300,300)
	# ax_3d.set_ylim(-300,300)
	# ax_3d.set_zlim(-300,100)
	# ax_3d.plot3D(XYZ_1[0], XYZ_1[1], XYZ_1[2], 'red')
	# ax_3d.plot3D(XYZ_2[0], XYZ_2[1], XYZ_2[2], 'green')
	# # ax_3d.plot3D(XYZ_3[0], XYZ_3[1], XYZ_3[2], 'blue')
	# # ax_3d.plot3D(XYZ_4[0], XYZ_4[1], XYZ_4[2], 'cyan')
	# # ax_3d.plot3D(XYZ_5[0], XYZ_5[1], XYZ_5[2], 'magenta')
	# # ax_3d.plot3D(XYZ_6[0], XYZ_6[1], XYZ_6[2], 'yellow')

	# fig2 = plt.figure(2)
	# fig2.suptitle('3D', fontsize=10)
	# ax2_3d = plt.axes(projection='3d')
	# ax2_3d.set_xlabel('x')
	# ax2_3d.set_ylabel('y')
	# ax2_3d.set_zlabel('z')
	# ax2_3d.set_xlim(-300,300)
	# ax2_3d.set_ylim(-300,300)
	# ax2_3d.set_zlim(-300,100)
	# # ax2_3d.plot3D(XYZ_4[0], XYZ_4[1], XYZ_4[2], 'red')
	# # ax2_3d.plot3D(XYZ_5[0], XYZ_5[1], XYZ_5[2], 'green')
	# ax2_3d.plot3D(XYZ_6[0], XYZ_6[1], XYZ_6[2], 'blue')

	# plt.show()