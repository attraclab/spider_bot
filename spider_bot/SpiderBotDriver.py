import time
import numpy as np
from spider_bot.dynamixel_sdk import *

### Dynamixel Addresses ###
ADDR_MODEL_NUMBER                = 0
ADDR_DRIVE_MODE                  = 10
ADDR_OPERATING_MODE              = 11
ADDR_CURRENT_LIMIT               = 38
ADDR_ACCELERATION_LIMIT          = 40
ADDR_VELOCITY_LIMIT              = 44
ADDR_TORQUE_ENABLE               = 64    
ADDR_POSITION_D_GAIN             = 80
ADDR_POSITION_I_GAIN             = 82
ADDR_POSITION_P_GAIN             = 84
ADDR_FEEDFORWARD_2nd_GAIN        = 88
ADDR_FEEDFORWARD_1st_GAIN        = 90
ADDR_GOAL_CURRENT                = 102
ADDR_GOAL_VELOCITY               = 104
ADDR_PROFILE_ACCELERATION        = 108		# VELOCITY BASED PROFILE
ADDR_PROFILE_VELOCITY            = 112		# VELOCITY BASED PROFILE
ADDR_PROFILE_ACCELERATION_TIME   = 108		# TIME BASED PROFILE
ADDR_PROFILE_TIME_SPAN           = 112      # TIME BASED PROFILE
ADDR_GOAL_POSITION               = 116
ADDR_MOVING                      = 122
ADDR_MOVING_STATUS               = 123
ADDR_PRESENT_CURRENT             = 126 
ADDR_PRESENT_POSITION            = 132

### Data Byte Length ###
LEN_GOAL_POSITION                = 4
LEN_PRESENT_POSITION             = 4
LEN_GOAL_CURRENT                 = 2
LEN_PRESENT_CURRENT              = 2
LEN_POS_TIME                     = 12
LEN_MOVING                       = 1
LEN_TORQUE_ENABLE                = 1	
LEN_CUR_VEL_POS                  = 10

### Operating Mode Number ###
CURRENT_CONTROL                      = 0
POSITION_CONTROL                     = 3 # Default
CURRENT_BASED_POSITION_CONTROL       = 5

### Velocity profile ###
TIME_BASED                           = 4
VELOCITY_BASED                       = 0

# Protocol version
PROTOCOL_VERSION = 2.0               

BAUDRATE = 1000000 #57600             
DEVICENAME = '/dev/u2d2' 
                         
TORQUE_ENABLE  = 1  
TORQUE_DISABLE = 0     

# ID
DXL1_ID  = 1                          
DXL2_ID  = 2                             
DXL3_ID  = 3                            
DXL4_ID  = 4
DXL5_ID  = 5
DXL6_ID  = 6
DXL7_ID  = 7
DXL8_ID  = 8
DXL9_ID  = 9
DXL10_ID = 10
DXL11_ID = 11
DXL12_ID = 12
DXL13_ID = 13
DXL14_ID = 14
DXL15_ID = 15
DXL16_ID = 16
DXL17_ID = 17
DXL18_ID = 18

CUR_UNIT = 2.69 # multiply this to read value will get value in mA

### Dynamixel Init ###
portHandler = PortHandler(DEVICENAME)

#exit()
# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)
print("Protocol Version {}".format(packetHandler.getProtocolVersion()))

# Open port
if portHandler.openPort():
	print("Succeeded to open the port")
else:
	print("Failed to open the port")
	print("Press any key to terminate...")
	getch()
	quit()

# Set port BAUDRATE
if portHandler.setBaudRate(BAUDRATE):
	print("Succeeded to change the BAUDRATE")
else:
	print("Failed to change the BAUDRATE")
	print("Press any key to terminate...")
	getch()
	quit()

# Initialize GroupSyncWrite instance for Goal Position
groupSyncWritePosition = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION,LEN_GOAL_POSITION)

# Initialize GroupSyncRead instace for Present Position
groupSyncReadPosition = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

groupSyncReadMoving = GroupSyncRead(portHandler, packetHandler, ADDR_MOVING, LEN_MOVING)

groupSyncReadTorqueEnable = GroupSyncRead(portHandler, packetHandler, ADDR_MOVING, LEN_TORQUE_ENABLE)

# # Initialize GroupSyncWrite instance for Goal Current
# self.groupSyncWriteCurrent = GroupSyncWrite(portHandler, packetHandler, self.ADDR_GOAL_CURRENT, self.LEN_GOAL_CURRENT)

# Initialize GroupSyncRead instace for Present Current, Velocity, Position; needs to modity group_sync_read.py
groupSyncReadCurVelPos = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_CURRENT, LEN_CUR_VEL_POS)

# Initialize GroupSyncRWrite instace for Time-based profile 
groupSyncWritePositionInTime = GroupSyncWrite(portHandler, packetHandler, ADDR_PROFILE_ACCELERATION, LEN_POS_TIME)

class SpiderBotDriver:

	def __init__(self):

		## Servo Parameters ##
		self.DriveMode = 0

		self.P_pose_Gain = 1000    #800 default
		self.I_pose_Gain = 0     #0 default
		self.D_pose_Gain = 4000   #4700 default

		self.P_current_Gain = 500    #800 default
		self.I_current_Gain = 0     #0 default
		self.D_current_Gain = 4000   #4700 default

		self.goal_current = 300

		self.TF = 50
		self.TA = self.TF//3

		self.min_deg = -180.0
		self.max_deg = 180.0
		self.min_bit = 0
		self.max_bit = 4095

		self.joint_deg_cmd = {
			1: 0.0,
			2: 0.0,
			3: 0.0,
			4: 0.0,
			5: 0.0,
			6: 0.0,
			7: 0.0,
			8: 0.0,
			9: 0.0,
			10: 0.0,
			11: 0.0,
			12: 0.0,
			13: 0.0,
			14: 0.0,
			15: 0.0,
			16: 0.0,
			17: 0.0,
			18: 0.0,
		}

		self.joint_moving = {
			1: 0,
			2: 0,
			3: 0,
			4: 0,
			5: 0,
			6: 0,
			7: 0,
			8: 0,
			9: 0,
			10: 0,
			11: 0,
			12: 0,
			13: 0,
			14: 0,
			15: 0,
			16: 0,
			17: 0,
			18: 0,
		}

		self.joint_position = {
			1: 0.0,
			2: 0.0,
			3: 0.0,
			4: 0.0,
			5: 0.0,
			6: 0.0,
			7: 0.0,
			8: 0.0,
			9: 0.0,
			10: 0.0,
			11: 0.0,
			12: 0.0,
			13: 0.0,
			14: 0.0,
			15: 0.0,
			16: 0.0,
			17: 0.0,
			18: 0.0,
		}

		self.joint_torque_enable = {
			1: 0,
			2: 0,
			3: 0,
			4: 0,
			5: 0,
			6: 0,
			7: 0,
			8: 0,
			9: 0,
			10: 0,
			11: 0,
			12: 0,
			13: 0,
			14: 0,
			15: 0,
			16: 0,
			17: 0,
			18: 0,
		}

		self.joint_cur_pos = {
			4: {'cur': 0.0, 'pos': 0.0},
			5: {'cur': 0.0, 'pos': 0.0},
			6: {'cur': 0.0, 'pos': 0.0},
			7: {'cur': 0.0, 'pos': 0.0},
			8: {'cur': 0.0, 'pos': 0.0},
			9: {'cur': 0.0, 'pos': 0.0}
		}

		self.total_servo = len(self.joint_deg_cmd)

		self.TorqueOff()

		self.servo_mode = 3

		## Setup as group ##
		for i in range(self.total_servo):

			_id = i+1

			## Add parameter storage for SyncRead
			# dxl_addparam_result = groupSyncReadPosition.addParam(_id)
			# if dxl_addparam_result != True:
			# 	print("Initialize: ERROR")
			# 	print("[ID:{:d}] groupSyncRead addparam failed".format(_id))
			# 	quit()

			## Add Params for GroupSyncRead ###
			self.AddParamSyncReadMoving(_id)
			self.AddParamSyncReadPosition(_id)
			self.AddParamSyncTorqueEnable(_id)

			## Only leg2 and leg3 servo 4 -> 9 needs to use for read current
			# if ((_id == 5) or (_id == 6) or (_id == 8) or (_id == 9)):
			if ((_id >= 4) and (_id <= 9)):
				self.AddParamSyncReadCurVelPos(_id)

			if self.servo_mode == 3:

				## Set operation mode ##
				self.SetOperatingMode(_id, POSITION_CONTROL)

				## Set driving mode ##
				self.SetDrivingMode(_id, TIME_BASED)

				self.SetTimeBaseProfile(_id, self.TF, self.TA)

				## Set PID gain ##
				self.SetPID(_id, self.P_pose_Gain, self.I_pose_Gain, self.D_pose_Gain)

			elif self.servo_mode == 5:
				## Set operation mode ##
				self.SetOperatingMode(_id, CURRENT_BASED_POSITION_CONTROL)

				self.SetGoalCurrent(_id, self.goal_current)

				## Set driving mode ##
				self.SetDrivingMode(_id, TIME_BASED)

				self.SetTimeBaseProfile(_id, self.TF, self.TA)

				## Set PID gain ##
				self.SetPID(_id, self.P_current_Gain, self.I_current_Gain, self.D_current_Gain)


	######################
	### Math Functions ###
	######################
	def map(self, val, in_min, in_max, out_min, out_max):   
		return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

	#######################
	### Setup Functions ###
	#######################
	def AddParamSyncReadMoving(self, ID):
		# Add parameter storage for Dynamixel#1 present position value
		dxl_addparam_result = groupSyncReadMoving.addParam(ID)
		if dxl_addparam_result != True:
			print("[ID:{:d}] groupSyncReadMoving addparam failed".format(ID))
			quit()

	def AddParamSyncReadPosition(self, ID):
		# Add parameter storage for Dynamixel#1 present position value
		dxl_addparam_result = groupSyncReadPosition.addParam(ID)
		if dxl_addparam_result != True:
			print("[ID:{:d}] groupSyncReadPosition addparam failed".format(ID))
			quit()

	def AddParamSyncTorqueEnable(self, ID):
		# Add parameter storage for Dynamixel#1 present position value
		dxl_addparam_result = groupSyncReadTorqueEnable.addParam(ID)
		if dxl_addparam_result != True:
			print("[ID:{:d}] groupSyncReadTorqueEnable addparam failed".format(ID))
			quit()

	def AddParamSyncReadCurVelPos(self, ID):
		# Add parameter storage for Dynamixel#1 present position value
		dxl_addparam_result = groupSyncReadCurVelPos.addParam(ID)
		if dxl_addparam_result != True:
			print("[ID:{:d}] groupSyncReadCurVelPos addparam failed".format(ID))
			quit()

	def SetOperatingMode(self, ID, MODE):

		## Must set to torque off before 

		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_OPERATING_MODE, MODE)
		if dxl_comm_result != COMM_SUCCESS:
			print("SetOperatingMode %s" % packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("SetOperatingMode %s" % packetHandler.getRxPacketError(dxl_error))


		present_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, ID, ADDR_OPERATING_MODE)
		if dxl_comm_result != COMM_SUCCESS:
			print("SetOperatingMode %s Read" % packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("SetOperatingMode %s Read" % packetHandler.getRxPacketError(dxl_error))
		else:
			if present_mode == 0:
			    # Current (Torque) Control Mode
				print("Motor " + str(ID) + " Operating Mode is Torque Control")
			elif present_mode == 3:
			    # Position Control Mode
				print("Motor " + str(ID) + " Operating Mode is Position Control")
			elif present_mode == 5:
			    # Current-based Position Control Mode
				print("Motor " + str(ID) + " Operating Mode is Current-based Position Control")
			else:
				print("In other Mode that didn't set!")

	def SetOperatingMode_byLeg(self, leg_no, MODE):

		if leg_no == 1:
			index_shift = 1
		elif leg_no == 2:
			index_shift = 4
		elif leg_no == 3:
			index_shift = 7
		elif leg_no == 4:
			index_shift = 10
		elif leg_no == 5:
			index_shift = 13
		elif leg_no == 6:
			index_shift = 16

		for i in range(3):
			servo_id = i+index_shift

			dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_OPERATING_MODE, MODE)
			if dxl_comm_result != COMM_SUCCESS:
				print("SetOperatingMode_byLeg %s" % packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				print("SetOperatingMode_byLeg %s" % packetHandler.getRxPacketError(dxl_error))


			present_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, servo_id, ADDR_OPERATING_MODE)
			if dxl_comm_result != COMM_SUCCESS:
				print("SetOperatingMode_byLeg %s" % packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				print("SetOperatingMode_byLeg %s" % packetHandler.getRxPacketError(dxl_error))
			else:
				if present_mode == 0:
				    # Current (Torque) Control Mode
					print("Motor " + str(servo_id) + " Operating Mode is Torque Control")
				elif present_mode == 3:
				    # Position Control Mode
					print("Motor " + str(servo_id) + " Operating Mode is Position Control")
				elif present_mode == 5:
				    # Current-based Position Control Mode
					print("Motor " + str(servo_id) + " Operating Mode is Current-based Position Control")
				else:
					print("In other Mode that didn't set!")


	def SetDrivingMode(self, ID, Base):
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_DRIVE_MODE, Base)

		self.DriveMode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, ID, ADDR_DRIVE_MODE)

		if self.DriveMode == 4:
			print("Motor " + str(ID) + " is in Driving Mode : Time-Based Profile")
		elif self.DriveMode == 0:
			print("Motor " + str(ID) + " is in Driving Mode : Velocity-Based Profile")
		else:
			print("Motor " + str(ID) + " is in Driving Mode : Unknown Drive Mode...")

	def SetDrivingMode_byLeg(self, leg_no):

		if leg_no == 1:
			index_shift = 1
		elif leg_no == 2:
			index_shift = 4
		elif leg_no == 3:
			index_shift = 7
		elif leg_no == 4:
			index_shift = 10
		elif leg_no == 5:
			index_shift = 13
		elif leg_no == 6:
			index_shift = 16

		for i in range(3):
			servo_id = i+index_shift
			dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_DRIVE_MODE, TIME_BASED)

			DriveMode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, servo_id, ADDR_DRIVE_MODE)

			if DriveMode == 4:
				print("Motor " + str(servo_id) + " is in Driving Mode : Time-Based Profile")
			elif DriveMode == 0:
				print("Motor " + str(servo_id) + " is in Driving Mode : Velocity-Based Profile")
			else:
				print("Motor " + str(servo_id) + " is in Driving Mode : Unknown Drive Mode...")

	def SetPID(self,ID,set_P_Gain,set_I_Gain,set_D_Gain):

		dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, ID, ADDR_POSITION_P_GAIN, set_P_Gain)
		dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, ID, ADDR_POSITION_I_GAIN, set_I_Gain)
		dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, ID, ADDR_POSITION_D_GAIN, set_D_Gain)

		print("Position P Gain of ID" + str(ID) + ": " + str(set_P_Gain))
		print("Position I Gain of ID" + str(ID) + ": " + str(set_I_Gain))
		print("Position D Gain of ID" + str(ID) + ": " + str(set_D_Gain))
		print("------------------------------")

	def SetPID_byLeg(self,leg_no,set_P_Gain,set_I_Gain,set_D_Gain):

		if leg_no == 1:
			index_shift = 1
		elif leg_no == 2:
			index_shift = 4
		elif leg_no == 3:
			index_shift = 7
		elif leg_no == 4:
			index_shift = 10
		elif leg_no == 5:
			index_shift = 13
		elif leg_no == 6:
			index_shift = 16

		for i in range(3):
			servo_id = i+index_shift
			dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_POSITION_P_GAIN, set_P_Gain)
			dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_POSITION_I_GAIN, set_I_Gain)
			dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_POSITION_D_GAIN, set_D_Gain)

			print("Position P Gain of ID" + str(servo_id) + ": " + str(set_P_Gain))
			print("Position I Gain of ID" + str(servo_id) + ": " + str(set_I_Gain))
			print("Position D Gain of ID" + str(servo_id) + ": " + str(set_D_Gain))
			print("------------------------------")

	def SetTimeBaseProfile(self, ID, set_Tf, set_Ta):

		if self.DriveMode == 4:
			dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ID, ADDR_PROFILE_ACCELERATION_TIME, int(set_Ta))
			dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ID, ADDR_PROFILE_TIME_SPAN, int(set_Tf))

			print("Finish Time of Motor " + str(ID) + " : " + str(set_Tf))
			print("Acceleration time of Motor " + str(ID) + " : " + str(set_Ta))
			print("--------------------------------")
		else:
			print("TIME PRFL ERROR : DriveMode is invalid")

	def SetGoalCurrent(self, ID, SetCur):

		dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, ID, ADDR_GOAL_CURRENT, SetCur)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % packetHandler.getRxPacketError(dxl_error))
		else:
			print("Goal Current of ID {:d} is {:d}".format(ID, SetCur))

	def SetGoalCurrent_byLeg(self, leg_no, joint1_cur, joint2_cur, joint3_cur):

		if leg_no == 1:
			index_shift = 1
		elif leg_no == 2:
			index_shift = 4
		elif leg_no == 3:
			index_shift = 7
		elif leg_no == 4:
			index_shift = 10
		elif leg_no == 5:
			index_shift = 13
		elif leg_no == 6:
			index_shift = 16

		for i in range(3):
			servo_id = i+index_shift

			if i == 0:
				SetCur = joint1_cur
			elif i == 1:
				SetCur = joint2_cur
			elif i == 2:
				SetCur = joint3_cur

			dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_CURRENT, SetCur)
			if dxl_comm_result != COMM_SUCCESS:
				print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				print("%s" % packetHandler.getRxPacketError(dxl_error))
			else:
				print("GoalCurrent_byLeg of ID {:d} is {:d}".format(servo_id, SetCur))

	def TorqueOn(self):
		for i in range(self.total_servo):
			servo_id = i+1
			dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

			if dxl_comm_result != COMM_SUCCESS:
				print("TorqueOn ID {:d} FAILED {}".format(servo_id, packetHandler.getTxRxResult(dxl_comm_result)))
			elif dxl_error != 0:
				print("TorqueOn ID {:d} Error {}".format(servo_id, packetHandler.getRxPacketError(dxl_error)))
			else:
				print("TorqueOn ID {:d} : Enabled".format(servo_id))

	def TorqueOff(self):
		for i in range(self.total_servo):
			servo_id = i+1
			dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

			if dxl_comm_result != COMM_SUCCESS:
				print("TorqueOff ID {:d} FAILED {}".format(servo_id, packetHandler.getTxRxResult(dxl_comm_result)))
			elif dxl_error != 0:
				print("TorqueOff ID {:d} Error {}".format(servo_id, packetHandler.getRxPacketError(dxl_error)))
			else:
				print("TorqueOff ID {:d} : Enabled".format(servo_id))

	def LegTorqueOn(self, leg_no):

		if leg_no == 1:
			index_shift = 1
		elif leg_no == 2:
			index_shift = 4
		elif leg_no == 3:
			index_shift = 7
		elif leg_no == 4:
			index_shift = 10
		elif leg_no == 5:
			index_shift = 13
		elif leg_no == 6:
			index_shift = 16

		for i in range(3):
			servo_id = i + index_shift
			dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

			if dxl_comm_result != COMM_SUCCESS:
				print("Leg {:d} TorqueOn ID {:d} FAILED {}".format(leg_no, servo_id, packetHandler.getTxRxResult(dxl_comm_result)))
			elif dxl_error != 0:
				print("Leg {:d} TorqueOn ID {:d} Error {}".format(leg_no, servo_id, packetHandler.getRxPacketError(dxl_error)))
			else:
				print("Leg {:d} TorqueOn ID {:d} : Enabled".format(leg_no, servo_id))

	def LegTorqueOff(self, leg_no):

		if leg_no == 1:
			index_shift = 1
		elif leg_no == 2:
			index_shift = 4
		elif leg_no == 3:
			index_shift = 7
		elif leg_no == 4:
			index_shift = 10
		elif leg_no == 5:
			index_shift = 13
		elif leg_no == 6:
			index_shift = 16

		for i in range(3):
			servo_id = i + index_shift
			dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

			if dxl_comm_result != COMM_SUCCESS:
				print("Leg {:d} TorqueOff ID {:d} FAILED {}".format(leg_no, servo_id, packetHandler.getTxRxResult(dxl_comm_result)))
			elif dxl_error != 0:
				print("Leg {:d} TorqueOff ID {:d} Error {}".format(leg_no, servo_id, packetHandler.getRxPacketError(dxl_error)))
			else:
				print("Leg {:d} TorqueOff ID {:d} : Enabled".format(leg_no, servo_id))



	###########################
	### Read Servo Function ###
	###########################
	def ReadMoving(self):
		"""
		SyncReadMoving causes some delay so if wanna use don't put it after SyncWrite loop
		it would cause the control motion slower
		"""

		dxl_comm_result = groupSyncReadMoving.txRxPacket()
		if dxl_comm_result != COMM_SUCCESS:
			print("ReadMoving: {:}".format(packetHandler.getTxRxResult(dxl_comm_result)))

		for i in range(self.total_servo):
			servo_id = i+1

			# Check if groupsyncread data of servo_id is available
			dxl_getdata_result = groupSyncReadMoving.isAvailable(servo_id, ADDR_MOVING, LEN_MOVING)
			if dxl_getdata_result != True:
				print("ReadMoving: ERROR")
				print("[ID:{:d}] groupSyncReadMoving getdata failed".format(servo_id))
				quit()

		for i in range(self.total_servo):
			servo_id = i+1
			# Get Dynamixel servo_id moving value
			moving = groupSyncReadMoving.getData(servo_id, ADDR_MOVING, LEN_MOVING)

			self.joint_moving[servo_id] = moving

	
	def ReadPosition(self):
		"""
		SyncReadPosition causes some delay so if wanna use don't put it after SyncWrite loop
		it would cause the control motion slower
		on Jetson Nano this function took 0.03 seconds to read
		"""

		dxl_comm_result = groupSyncReadPosition.txRxPacket()
		if dxl_comm_result != COMM_SUCCESS:
			print("ReadPosition: {:}".format(packetHandler.getTxRxResult(dxl_comm_result)))


		for i in range(self.total_servo):
			servo_id = i+1
			# Check if groupsyncread data of servo_id is available
			dxl_getdata_result = groupSyncReadPosition.isAvailable(servo_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
			if dxl_getdata_result != True:
				print("ReadPosition: ERROR")
				print("[ID:{:d}] groupSyncReadPosition getdata failed".format(servo_id))
				# quit()

			raw_position = groupSyncReadPosition.getData(servo_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
			self.joint_position[servo_id] = self.map(raw_position, 0.0, 4095.0, -180.0, 180.0)

		# for i in range(self.total_servo):
		# 	servo_id = i+1
		# 	# Get Dynamixel servo_id moving value
		# 	raw_position = groupSyncReadPosition.getData(servo_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

		# 	self.joint_position[servo_id] = self.map(raw_position, 0.0, 4095.0, -180.0, 180.0)

	def ReadTorqueEnable(self):
		dxl_comm_result = groupSyncReadTorqueEnable.txRxPacket()
		if dxl_comm_result != COMM_SUCCESS:
			print("ReadTorqueEnable: {:}".format(packetHandler.getTxRxResult(dxl_comm_result)))


		for i in range(self.total_servo):
			servo_id = i+1
			# Check if groupsyncread data of servo_id is available
			dxl_getdata_result = groupSyncReadTorqueEnable.isAvailable(servo_id, ADDR_TORQUE_ENABLE, LEN_TORQUE_ENABLE)
			if dxl_getdata_result != True:
				print("ReadTorqueEnable: ERROR")
				print("[ID:{:d}] groupSyncReadTorqueEnable getdata failed".format(servo_id))
				quit()

		for i in range(self.total_servo):
			servo_id = i+1
			# Get Dynamixel servo_id moving value
			torque_enable = groupSyncReadTorqueEnable.getData(servo_id, ADDR_TORQUE_ENABLE, LEN_TORQUE_ENABLE)

			self.joint_torque_enable[servo_id] = torque_enable

	def ReadCurrentPosition(self):
		"""
		Read Curent, Velocity, and Position of servo 5,6,8,9
		The current is in mA and position is in degrees, but velocity is ignore.
		The value will be stored in dict joint_cur_pose[servo_id]
		on Jetson Nano this function took 0.01 seconds to read
		"""

		dxl_comm_result = groupSyncReadCurVelPos.txRxPacket()
		if dxl_comm_result != COMM_SUCCESS:
			print("ReadTorquePosition: {:}".format(packetHandler.getTxRxResult(dxl_comm_result)))


		for i in range(6):
			## we focus only servo 5,6  8,9 skip 7
			servo_id = i + 4
			# if servo_id == 7:
			# 	continue

			dxl_getdata_result = groupSyncReadCurVelPos.isAvailable(servo_id, ADDR_PRESENT_CURRENT, LEN_CUR_VEL_POS)
			if dxl_getdata_result != True:
				print("ReadPosition: ERROR")
				print("[ID:{:d}] groupSyncReadCurVelPos getdata failed".format(servo_id))

			raw_cur, raw_vel, raw_pos = groupSyncReadCurVelPos.getData(servo_id, ADDR_PRESENT_CURRENT, LEN_CUR_VEL_POS)

			raw_cur_sign = np.int16(raw_cur)
			cur_mA = raw_cur_sign*CUR_UNIT
			deg = self.map(raw_pos, 0.0, 4095.0, -180.0, 180.0)

			self.joint_cur_pos[servo_id]['cur'] = cur_mA
			self.joint_cur_pos[servo_id]['pos'] = deg




	#############################
	### Drive Servo Functions ###
	#############################
	def RunServo(self):

		for i in range(self.total_servo):
			servo_id = i+1

			servo_ang = int(self.map(self.joint_deg_cmd[servo_id], self.min_deg, self.max_deg, self.min_bit, self.max_bit))
			param_goal_position = [DXL_LOBYTE(DXL_LOWORD(servo_ang)), DXL_HIBYTE(DXL_LOWORD(servo_ang)), DXL_LOBYTE(DXL_HIWORD(servo_ang)), DXL_HIBYTE(DXL_HIWORD(servo_ang))]

			# Add Dynamixel goal position value to the Syncwrite parameter storage
			dxl_addparam_result = groupSyncWritePosition.addParam(servo_id, param_goal_position)
			if dxl_addparam_result != True:
				print("RunServo: ERROR")
				print("[ID:{:d}] groupSyncWrite addparam failed".format(servo_id))
				portHandler.closePort()
				quit()

		# Syncwrite goal position
		dxl_comm_result = groupSyncWritePosition.txPacket()
		if dxl_comm_result != COMM_SUCCESS:
			print("RunServo: ERROR")
			print("{}".format(packetHandler.getTxRxResult(dxl_comm_result)))

		# Clear syncwrite parameter storage
		groupSyncWritePosition.clearParam()

	def RunServoInTime(self, acc_time, finish_time):

		for i in range(self.total_servo):
			servo_id = i+1

			servo_ang = int(self.map(self.joint_deg_cmd[servo_id], self.min_deg, self.max_deg, self.min_bit, self.max_bit))
			acc_time = int(acc_time)
			finish_time = int(finish_time)
			time_position_data = [DXL_LOBYTE(DXL_LOWORD(acc_time)),
						DXL_HIBYTE(DXL_LOWORD(acc_time)),
						DXL_LOBYTE(DXL_HIWORD(acc_time)), 
						DXL_HIBYTE(DXL_HIWORD(acc_time)),
						DXL_LOBYTE(DXL_LOWORD(finish_time)), 
						DXL_HIBYTE(DXL_LOWORD(finish_time)),
						DXL_LOBYTE(DXL_HIWORD(finish_time)), 
						DXL_HIBYTE(DXL_HIWORD(finish_time)),
						DXL_LOBYTE(DXL_LOWORD(servo_ang)), 
						DXL_HIBYTE(DXL_LOWORD(servo_ang)),
						DXL_LOBYTE(DXL_HIWORD(servo_ang)), 
						DXL_HIBYTE(DXL_HIWORD(servo_ang))]

			# Add Dynamixel goal position value to the Syncwrite parameter storage
			dxl_addparam_result = groupSyncWritePositionInTime.addParam(servo_id, time_position_data)
			if dxl_addparam_result != True:
				print("RunServoInTime: ERROR")
				print("[ID:{:d}] groupSyncWritePositionInTime addparam failed".format(servo_id))
				portHandler.closePort()
				quit()

		# Syncwrite goal position
		dxl_comm_result = groupSyncWritePositionInTime.txPacket()
		if dxl_comm_result != COMM_SUCCESS:
			print("RunServoInTime: ERROR")
			print("{}".format(packetHandler.getTxRxResult(dxl_comm_result)))

		# Clear syncwrite parameter storage
		groupSyncWritePositionInTime.clearParam()

	def RunServoInTimeByLeg(self, acc_time, finish_time, leg_no):

		if leg_no == 1:
			start_id = 1

		elif leg_no == 2:
			start_id = 4

		elif leg_no == 3:
			start_id = 7

		elif leg_no == 4:
			start_id = 10

		elif leg_no == 5:
			start_id = 13

		elif leg_no == 6:
			start_id = 16

		else:
			print("RunServoInTimeByLeg ERROR invalid leg_no {}".format(leg_no))
			return

		for i in range(3):
			servo_id = start_id + i
			servo_ang = int(self.map(self.joint_deg_cmd[servo_id], self.min_deg, self.max_deg, self.min_bit, self.max_bit))
			acc_time = int(acc_time)
			finish_time = int(finish_time)
			time_position_data = [DXL_LOBYTE(DXL_LOWORD(acc_time)),
						DXL_HIBYTE(DXL_LOWORD(acc_time)),
						DXL_LOBYTE(DXL_HIWORD(acc_time)), 
						DXL_HIBYTE(DXL_HIWORD(acc_time)),
						DXL_LOBYTE(DXL_LOWORD(finish_time)), 
						DXL_HIBYTE(DXL_LOWORD(finish_time)),
						DXL_LOBYTE(DXL_HIWORD(finish_time)), 
						DXL_HIBYTE(DXL_HIWORD(finish_time)),
						DXL_LOBYTE(DXL_LOWORD(servo_ang)), 
						DXL_HIBYTE(DXL_LOWORD(servo_ang)),
						DXL_LOBYTE(DXL_HIWORD(servo_ang)), 
						DXL_HIBYTE(DXL_HIWORD(servo_ang))]

			# Add Dynamixel goal position value to the Syncwrite parameter storage
			dxl_addparam_result = groupSyncWritePositionInTime.addParam(servo_id, time_position_data)
			if dxl_addparam_result != True:
				print("RunServoInTimeByLeg: ERROR")
				print("[ID:{:d}] groupSyncWritePositionInTime addparam failed".format(servo_id))
				portHandler.closePort()
				quit()

		# Syncwrite goal position
		dxl_comm_result = groupSyncWritePositionInTime.txPacket()
		if dxl_comm_result != COMM_SUCCESS:
			print("RunServoInTimeByLeg: ERROR")
			print("{}".format(packetHandler.getTxRxResult(dxl_comm_result)))

		# Clear syncwrite parameter storage
		groupSyncWritePositionInTime.clearParam()