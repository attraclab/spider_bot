import rclpy
from rclpy.node import Node
import time
import numpy as np
from std_msgs.msg import Int16MultiArray, Int8, UInt8MultiArray, Float32MultiArray, String, Bool, Float32, UInt8
from spider_bot.SpiderBotLib import *
from spider_bot.SpiderBotDriver import *
import joblib
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

#############################
### Setup SpiderBotDriver ###
#############################
driver = SpiderBotDriver()


##########################
### Setup SpiderBotLib ###
##########################
lib = SpiderBotLib()
lib.generate_crabWalkingLUT()
DATA_POINT_ALL = lib.DATA_POINT_ALL
DATA_POINT_TURN_ALL = lib.data_point_TURN_ALL
crabwalk_angle_length = lib.cw_LUT_increment
cbw_ang_res = lib.cw_ang_resolution
WG_DATA_POINT_ALL = lib.WG_DATA_POINT_ALL

#################
### SVM Model ###
#################
svm_model = joblib.load('/home/jetson/dev_ws/src/spider_bot/spider_bot/svm_models/20240123_641dataset.joblib')

class SpiderBotControl(Node):

	def __init__(self):

		super().__init__('spider_bot_control')
		self.get_logger().info('Start spider_bot_control node')

		### ROS parameters ###
		self.declare_parameter('show_log', True)

		self.add_on_set_parameters_callback(self.parameter_callback)

		self.show_log = self.get_parameter('show_log').get_parameter_value().bool_value

		self.get_logger().info("Using parameters as below")
		self.get_logger().info("show_log: {}".format(self.show_log))

		### Standby movement ###
		self.coxa_pre_home_ang = 0.0
		self.femur_pre_home_ang = 45.0
		self.tibia_pre_home_ang = -45.0

		self.coxa_home_ang = np.degrees(lib.theta1_home)
		self.femur_home_ang = np.degrees(lib.theta2_home)
		self.tibia_home_ang = np.degrees(lib.theta3_home)

		self.standby_movement_time = 1000

		self.CUSTOM_HOME_LEG = {
			1: {'theta1': lib.theta1_home, 'theta2': lib.theta2_home, 'theta3': lib.theta3_home,},
			2: {'theta1': lib.theta1_home, 'theta2': lib.theta2_home, 'theta3': lib.theta3_home,},
			3: {'theta1': lib.theta1_home, 'theta2': lib.theta2_home, 'theta3': lib.theta3_home,},
			4: {'theta1': lib.theta1_home, 'theta2': lib.theta2_home, 'theta3': lib.theta3_home,},
			5: {'theta1': lib.theta1_home, 'theta2': lib.theta2_home, 'theta3': lib.theta3_home,},
			6: {'theta1': lib.theta1_home, 'theta2': lib.theta2_home, 'theta3': lib.theta3_home,},
		}

		self.CUSTOM_HEIGHT_HOME_LEG = {
			1: {'theta1': lib.theta1_home, 'theta2': lib.theta2_home, 'theta3': lib.theta3_home,},
			2: {'theta1': lib.theta1_home, 'theta2': lib.theta2_home, 'theta3': lib.theta3_home,},
			3: {'theta1': lib.theta1_home, 'theta2': lib.theta2_home, 'theta3': lib.theta3_home,},
			4: {'theta1': lib.theta1_home, 'theta2': lib.theta2_home, 'theta3': lib.theta3_home,},
			5: {'theta1': lib.theta1_home, 'theta2': lib.theta2_home, 'theta3': lib.theta3_home,},
			6: {'theta1': lib.theta1_home, 'theta2': lib.theta2_home, 'theta3': lib.theta3_home,},
		}

		self.customHeight_home_xyz = [lib.X_home, lib.Y_home, lib.Z_home]

		driver.TorqueOn()
		self.standByPosition()

		#### Crab-walking
		self.walk_counter = 0
		self.smallest_delay = 20   # ms
		self.biggest_delay = 80    # ms
		self.last_crab_walk_stamp = time.time()
		self.delay_cb_time = self.biggest_delay
		self.ang_index = 0
		self.full_front_idx = 0
		self.full_left_idx = crabwalk_angle_length//4
		self.full_right_idx = int(3*crabwalk_angle_length//4)
		self.full_back_idx = crabwalk_angle_length//2
		self.cw_LUT_last_idx = crabwalk_angle_length-1

		#### In-place turning
		self.turning_angle = 30
		self.TURN_CCW_LEG = {
			1: {'theta1': [], 'theta2': [], 'theta3': [],}, 
			2: {'theta1': [], 'theta2': [], 'theta3': [],}, 
			3: {'theta1': [], 'theta2': [], 'theta3': [],}, 
			4: {'theta1': [], 'theta2': [], 'theta3': [],}, 
			5: {'theta1': [], 'theta2': [], 'theta3': [],}, 
			6: {'theta1': [], 'theta2': [], 'theta3': [],},  
		}
		self.TURN_CW_LEG = {
			1: {'theta1': [], 'theta2': [], 'theta3': [],},
			2: {'theta1': [], 'theta2': [], 'theta3': [],}, 
			3: {'theta1': [], 'theta2': [], 'theta3': [],}, 
			4: {'theta1': [], 'theta2': [], 'theta3': [],}, 
			5: {'theta1': [], 'theta2': [], 'theta3': [],}, 
			6: {'theta1': [], 'theta2': [], 'theta3': [],},  
		}
		self.TURN_CUSTOM_CCW_LEG = {
			1: {'theta1': [], 'theta2': [], 'theta3': [],},
			2: {'theta1': [], 'theta2': [], 'theta3': [],}, 
			3: {'theta1': [], 'theta2': [], 'theta3': [],}, 
			4: {'theta1': [], 'theta2': [], 'theta3': [],}, 
			5: {'theta1': [], 'theta2': [], 'theta3': [],}, 
			6: {'theta1': [], 'theta2': [], 'theta3': [],},  
		}
		self.TURN_CUSTOM_CW_LEG = {
			1: {'theta1': [], 'theta2': [], 'theta3': [],},
			2: {'theta1': [], 'theta2': [], 'theta3': [],}, 
			3: {'theta1': [], 'theta2': [], 'theta3': [],}, 
			4: {'theta1': [], 'theta2': [], 'theta3': [],}, 
			5: {'theta1': [], 'theta2': [], 'theta3': [],}, 
			6: {'theta1': [], 'theta2': [], 'theta3': [],},  
		}

		legIKM_CCW_XYZ, legJLN_CCW_XYZ = lib.generate_inplace_turning(-self.turning_angle)
		self.TURN_CCW_LEG[1]['theta1'], self.TURN_CCW_LEG[1]['theta2'], self.TURN_CCW_LEG[1]['theta3'] = lib.invKinArray_to_ThetaArray(legIKM_CCW_XYZ)
		self.TURN_CCW_LEG[3]['theta1'], self.TURN_CCW_LEG[3]['theta2'], self.TURN_CCW_LEG[3]['theta3'] = self.TURN_CCW_LEG[1]['theta1'], self.TURN_CCW_LEG[1]['theta2'], self.TURN_CCW_LEG[1]['theta3']
		self.TURN_CCW_LEG[5]['theta1'], self.TURN_CCW_LEG[5]['theta2'], self.TURN_CCW_LEG[5]['theta3'] = self.TURN_CCW_LEG[1]['theta1'], self.TURN_CCW_LEG[1]['theta2'], self.TURN_CCW_LEG[1]['theta3']
		self.TURN_CCW_LEG[2]['theta1'], self.TURN_CCW_LEG[2]['theta2'], self.TURN_CCW_LEG[2]['theta3'] = lib.invKinArray_to_ThetaArray(legJLN_CCW_XYZ)
		self.TURN_CCW_LEG[4]['theta1'], self.TURN_CCW_LEG[4]['theta2'], self.TURN_CCW_LEG[4]['theta3'] = self.TURN_CCW_LEG[2]['theta1'], self.TURN_CCW_LEG[2]['theta2'], self.TURN_CCW_LEG[2]['theta3']
		self.TURN_CCW_LEG[6]['theta1'], self.TURN_CCW_LEG[6]['theta2'], self.TURN_CCW_LEG[6]['theta3'] = self.TURN_CCW_LEG[2]['theta1'], self.TURN_CCW_LEG[2]['theta2'], self.TURN_CCW_LEG[2]['theta3']


		legIKM_CW_XYZ, legJLN_CW_XYZ = lib.generate_inplace_turning(self.turning_angle)
		self.TURN_CW_LEG[1]['theta1'], self.TURN_CW_LEG[1]['theta2'], self.TURN_CW_LEG[1]['theta3'] = lib.invKinArray_to_ThetaArray(legIKM_CW_XYZ)
		self.TURN_CW_LEG[3]['theta1'], self.TURN_CW_LEG[3]['theta2'], self.TURN_CW_LEG[3]['theta3'] = self.TURN_CW_LEG[1]['theta1'], self.TURN_CW_LEG[1]['theta2'], self.TURN_CW_LEG[1]['theta3']
		self.TURN_CW_LEG[5]['theta1'], self.TURN_CW_LEG[5]['theta2'], self.TURN_CW_LEG[5]['theta3'] = self.TURN_CW_LEG[1]['theta1'], self.TURN_CW_LEG[1]['theta2'], self.TURN_CW_LEG[1]['theta3']
		self.TURN_CW_LEG[2]['theta1'], self.TURN_CW_LEG[2]['theta2'], self.TURN_CW_LEG[2]['theta3'] = lib.invKinArray_to_ThetaArray(legJLN_CW_XYZ)
		self.TURN_CW_LEG[4]['theta1'], self.TURN_CW_LEG[4]['theta2'], self.TURN_CW_LEG[4]['theta3'] = self.TURN_CW_LEG[2]['theta1'], self.TURN_CW_LEG[2]['theta2'], self.TURN_CW_LEG[2]['theta3']
		self.TURN_CW_LEG[6]['theta1'], self.TURN_CW_LEG[6]['theta2'], self.TURN_CW_LEG[6]['theta3'] = self.TURN_CW_LEG[2]['theta1'], self.TURN_CW_LEG[2]['theta2'], self.TURN_CW_LEG[2]['theta3']

		self.TURN_CUSTOM_CCW_LEG[1]['theta1'], self.TURN_CUSTOM_CCW_LEG[1]['theta2'], self.TURN_CUSTOM_CCW_LEG[1]['theta3'] = self.TURN_CCW_LEG[1]['theta1'], self.TURN_CCW_LEG[1]['theta2'], self.TURN_CCW_LEG[1]['theta3']
		self.TURN_CUSTOM_CCW_LEG[3]['theta1'], self.TURN_CUSTOM_CCW_LEG[3]['theta2'], self.TURN_CUSTOM_CCW_LEG[3]['theta3'] = self.TURN_CCW_LEG[1]['theta1'], self.TURN_CCW_LEG[1]['theta2'], self.TURN_CCW_LEG[1]['theta3']
		self.TURN_CUSTOM_CCW_LEG[5]['theta1'], self.TURN_CUSTOM_CCW_LEG[5]['theta2'], self.TURN_CUSTOM_CCW_LEG[5]['theta3'] = self.TURN_CCW_LEG[1]['theta1'], self.TURN_CCW_LEG[1]['theta2'], self.TURN_CCW_LEG[1]['theta3']
		self.TURN_CUSTOM_CCW_LEG[2]['theta1'], self.TURN_CUSTOM_CCW_LEG[2]['theta2'], self.TURN_CUSTOM_CCW_LEG[2]['theta3'] = self.TURN_CCW_LEG[2]['theta1'], self.TURN_CCW_LEG[2]['theta2'], self.TURN_CCW_LEG[2]['theta3']
		self.TURN_CUSTOM_CCW_LEG[4]['theta1'], self.TURN_CUSTOM_CCW_LEG[4]['theta2'], self.TURN_CUSTOM_CCW_LEG[4]['theta3'] = self.TURN_CCW_LEG[2]['theta1'], self.TURN_CCW_LEG[2]['theta2'], self.TURN_CCW_LEG[2]['theta3']
		self.TURN_CUSTOM_CCW_LEG[6]['theta1'], self.TURN_CUSTOM_CCW_LEG[6]['theta2'], self.TURN_CUSTOM_CCW_LEG[6]['theta3'] = self.TURN_CCW_LEG[2]['theta1'], self.TURN_CCW_LEG[2]['theta2'], self.TURN_CCW_LEG[2]['theta3']

		self.TURN_CUSTOM_CW_LEG[1]['theta1'], self.TURN_CUSTOM_CW_LEG[1]['theta2'], self.TURN_CUSTOM_CW_LEG[1]['theta3'] = self.TURN_CW_LEG[1]['theta1'], self.TURN_CW_LEG[1]['theta2'], self.TURN_CW_LEG[1]['theta3']
		self.TURN_CUSTOM_CW_LEG[3]['theta1'], self.TURN_CUSTOM_CW_LEG[3]['theta2'], self.TURN_CUSTOM_CW_LEG[3]['theta3'] = self.TURN_CW_LEG[1]['theta1'], self.TURN_CW_LEG[1]['theta2'], self.TURN_CW_LEG[1]['theta3']
		self.TURN_CUSTOM_CW_LEG[5]['theta1'], self.TURN_CUSTOM_CW_LEG[5]['theta2'], self.TURN_CUSTOM_CW_LEG[5]['theta3'] = self.TURN_CW_LEG[1]['theta1'], self.TURN_CW_LEG[1]['theta2'], self.TURN_CW_LEG[1]['theta3']
		self.TURN_CUSTOM_CW_LEG[2]['theta1'], self.TURN_CUSTOM_CW_LEG[2]['theta2'], self.TURN_CUSTOM_CW_LEG[2]['theta3'] = self.TURN_CW_LEG[2]['theta1'], self.TURN_CW_LEG[2]['theta2'], self.TURN_CW_LEG[2]['theta3']
		self.TURN_CUSTOM_CW_LEG[4]['theta1'], self.TURN_CUSTOM_CW_LEG[4]['theta2'], self.TURN_CUSTOM_CW_LEG[4]['theta3'] = self.TURN_CW_LEG[2]['theta1'], self.TURN_CW_LEG[2]['theta2'], self.TURN_CW_LEG[2]['theta3']
		self.TURN_CUSTOM_CW_LEG[6]['theta1'], self.TURN_CUSTOM_CW_LEG[6]['theta2'], self.TURN_CUSTOM_CW_LEG[6]['theta3'] = self.TURN_CW_LEG[2]['theta1'], self.TURN_CW_LEG[2]['theta2'], self.TURN_CW_LEG[2]['theta3'] 

		self.turn_counter = 0
		self.last_turn_stamp = time.time()
		self.delay_turning_time = 40

		#### Reset Motion
		self.X_reset = lib.X_home
		self.Y_reset = lib.Y_home
		self.Z_reset = lib.Z_home - (-50.0)
		self.theta1_reset, self.theta2_reset, self.theta3_reset = lib.inv(self.X_reset, self.Y_reset, self.Z_reset)
		self.allow_reset = False
		self.reset_timeout = 0.1
		self.reset_movement_time = 50
		self.reset_moving = False
		self.home_moving = False
		self.last_reset_movement_stamp = time.time()
		self.last_home_movement_stamp = time.time()
		self.reset_leg_seq_list = [] #[1, 3, 5, 2, 4, 6]
		self.reset_seq_counter = 0
		self.leg_no = 1
		self.start_id = 1
		self.allow_check_legs_fwd = True

		#### Body control
		self.roll_cmd_deg = 0.0
		self.pitch_cmd_deg = 0.0
		self.yaw_cmd_deg = 0.0
		self.max_body_rotate_deg = 20.0
		self.max_yaw_rotate_deg = 40.0
		self.delay_body_rotaiton = 100
		self.last_body_rotation_stamp = time.time()

		### JMOAB Parameterers ###
		self.sbus_thr = 1024
		self.sbus_str = 1024
		self.sbus_roll = 1024
		self.sbus_pitch = 1024
		self.sbus_yaw = 1024
		self.sbus_max = 1680
		self.sbus_min = 368
		self.sbus_mid = 1024
		self.sbus_min_db = self.sbus_mid - 40
		self.sbus_max_db = self.sbus_mid + 40

		self.cart_mode = 1
		self.robot_mode = "CRAB"
		self.movement_mode = "crabwalk"

		### Web control ###
		self.control_mode = 0 # 0: flat-walking, 1: orientation, 2: rough-walking, 3: inspection
		self.prev_control_mode = 0
		self.thr_axis = 0.0
		self.str_axis = 0.0
		self.turn_axis = 0.0
		self.move_norm = 0.0
		self.move_ang = 0.0
		self.height_cmd = 0.0
		self.pitch_cmd = 0.0
		self.prev_pitch_cmd = self.pitch_cmd
		self.walk_custom_counter = 0
		self.last_crab_walk_custom_stamp = time.time()

		self.height_pitch_cmd_time = 1000

		### inspection mode ###
		self.inspect_standby = False
		self.leg2_x, self.leg2_y, self.leg2_z = 0.0, 0.0, 0.0
		self.leg3_x, self.leg3_y, self.leg3_z = 0.0, 0.0, 0.0
		self.got_leg2_xyz_cmd = False
		self.got_leg3_xyz_cmd = False
		self.insp_leg_no = 2
		self.inspect_movement_time = 500
		self.last_read_fb_stamp = time.time()
		self.got_height_cmd_inspection = False
		self.got_inspect_start = False
		self.insp_step = 0
		self.z_touch_counter = 0
		self.z_touch_limit = 50
		self.inspect_touch_time = 100
		self.z_inspect_counter = 0
		self.z_inspect_limit = 20
		self.insp_sv_list = [4,5,6]

		### Wave-Gait Rough walking ###
		self.delay_wg_time = time.time()
		self.wg_walk_counter = 0
		self.wg_smallest_delay = 20 #20   # ms
		self.wg_biggest_delay = 80 #80    # ms
		self.wg_constant_delay = 70
		self.last_wave_gait_stamp = time.time()
		self.prev_ang_index = self.ang_index

		### Pub/Sub ###
		self.sbus_rc_sub = self.create_subscription(Int16MultiArray, '/jmoab/sbus_rc_ch', self.sbus_rc_callback, 10)
		self.cart_mode_sub = self.create_subscription(UInt8, "/jmoab/cart_mode", self.cart_mode_callback, 10)
		
		self.console_joy_sub = self.create_subscription(Float32MultiArray, '/console/joy', self.console_joy_callback, 10)
		self.control_mode_sub = self.create_subscription(Int8, "/spider/control_mode_cmd", self.control_mode_cmd_callback, 10)
		self.height_cmd_sub = self.create_subscription(Float32, "/spider/height_cmd", self.height_cmd_callback, 10)
		self.pitch_cmd_sub = self.create_subscription(Float32, "/spider/pitch_cmd", self.pitch_cmd_callback, 10)
		self.reset_height_cmd_sub = self.create_subscription(Bool, "/spider/reset_height", self.reset_height_callback, 10)
		self.reset_ppitch_cmd_sub = self.create_subscription(Bool, "/spider/reset_pitch", self.reset_pitch_callback, 10)

		self.leg2_xyz_fb_pub = self.create_publisher(Float32MultiArray, "/spider/leg2_xyz_fb", 10)
		self.leg3_xyz_fb_pub = self.create_publisher(Float32MultiArray, "/spider/leg3_xyz_fb", 10)

		self.leg2_xyz_cmd_sub = self.create_subscription(Float32MultiArray, "/spider/leg2_xyz_cmd", self.leg2_xyz_cmd_callback, 10)
		self.leg3_xyz_cmd_sub = self.create_subscription(Float32MultiArray, "/spider/leg3_xyz_cmd", self.leg3_xyz_cmd_callback, 10)

		self.inspect_start_sub = self.create_subscription(Bool, "/spider/inspection_start", self.inspection_start_callback, 10)
		self.inspect_status_pub = self.create_publisher(Int8, "/spider/inspection_status", 10)

		self.train_data_pub = self.create_publisher(Float32MultiArray, '/spider/train_data', 10)

		### Loop ###
		self.from_init = True
		timer_period = 0.01
		self.timer = self.create_timer(timer_period, self.timer_callback)

	def _print(self, msg):
		self.get_logger().info(msg)


	#####################
	### ROS callbacks ###
	#####################
	def parameter_callback(self, params):
		for param in params:
			if (param.name == 'show_log') and (param.type_ == Parameter.Type.BOOL):
				self.show_log = param.value

		self.get_logger().info("Updated parameter")

	def sbus_rc_callback(self, msg):

		self.sbus_str = msg.data[0]
		self.sbus_thr = msg.data[1]

		self.sbus_roll =  msg.data[1]
		self.sbus_pitch = msg.data[3]
		self.sbus_yaw = msg.data[0]

		if msg.data[6] < 1000:
			self.movement_mode = 'crabwalk'
		elif msg.data[6] < 1500:
			self.movement_mode = 'bodycontrol'
		else:
			self.movement_mode = None

	def cart_mode_callback(self, msg):

		self.cart_mode = msg.data


	def control_mode_cmd_callback(self, msg):

		self.prev_control_mode = self.control_mode
		self.control_mode = msg.data
		self._print("prev_control_mode {}".format(self.prev_control_mode))
		self._print("control_mode {}".format(self.control_mode))

		if (self.control_mode == 2) and (self.prev_control_mode != self.control_mode):
			self._print("change to current-base control")
			angle_list = [
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						]
			self.setServoAngle(angle_list)

			coxa_current = 200
			femur_current = 150
			tibia_current = 150

			acc_time = 10
			finish_time = 50
			sleep_time = 0.05

			P = 300
			I = 10
			D = 2000

			for i in range(6):

				leg_no = i + 1

				driver.LegTorqueOff(leg_no)
				driver.SetOperatingMode_byLeg(leg_no, 5)
				driver.SetPID_byLeg(leg_no, P, I, D)
				driver.SetGoalCurrent_byLeg(leg_no, coxa_current, femur_current, tibia_current)
				driver.LegTorqueOn(leg_no)
				driver.RunServoInTimeByLeg(acc_time=acc_time, finish_time=finish_time, leg_no=leg_no)
				time.sleep(sleep_time)

			self.setBodyHeight(80.0)
			self.height_cmd = 80.0

		elif ((self.control_mode == 0) or (self.control_mode == 1)) and (self.prev_control_mode != self.control_mode) and ((self.prev_control_mode == 2) or (self.prev_control_mode == 3)):
			self._print("change to position control")

			## if came from Rough-Walking, need to lower the height back first.
			if self.prev_control_mode == 2:
				self.setBodyHeight(0.0)
				self.height_cmd = 0.0

			angle_list = [
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						]
			self.setServoAngle(angle_list)

			acc_time = 10
			finish_time = 50
			sleep_time = 0.05

			P = 1000
			I = 0
			D = 4000

			if self.prev_control_mode == 2:
				## change mode on all legs
				for i in range(6):
					leg_no = i + 1

					driver.LegTorqueOff(leg_no)
					driver.SetOperatingMode_byLeg(leg_no, 3)
					driver.SetPID_byLeg(leg_no, P, I, D)
					driver.LegTorqueOn(leg_no)
					driver.RunServoInTimeByLeg(acc_time=acc_time, finish_time=finish_time, leg_no=leg_no)
					time.sleep(sleep_time)
			elif self.prev_control_mode == 3:
				## reset the motion first because posture is in inspection
				self.resetMotion_toHome()
				## change mode back only leg2, leg3
				for i in range(2):
					leg_no = i + 2

					driver.LegTorqueOff(leg_no)
					driver.SetOperatingMode_byLeg(leg_no, 3)
					driver.SetPID_byLeg(leg_no, P, I, D)
					driver.LegTorqueOn(leg_no)
					driver.RunServoInTimeByLeg(acc_time=acc_time, finish_time=finish_time, leg_no=leg_no)
					time.sleep(sleep_time)

		elif (self.control_mode == 3) and (self.prev_control_mode != self.control_mode):
			self._print("chang to inspectio mode")

			coxa_current = 500
			femur_current = 500
			tibia_current = 500

			acc_time = 10
			finish_time = 50
			sleep_time = 0.05

			P = 300
			I = 10
			D = 2000

			for i in range(2):
				leg_no = i + 2

				driver.LegTorqueOff(leg_no)
				driver.SetOperatingMode_byLeg(leg_no, 5)
				driver.SetPID_byLeg(leg_no, P, I, D)
				driver.SetGoalCurrent_byLeg(leg_no, coxa_current, femur_current, tibia_current)
				driver.LegTorqueOn(leg_no)
				driver.RunServoInTimeByLeg(acc_time=acc_time, finish_time=finish_time, leg_no=leg_no)
				time.sleep(sleep_time)

			self.setInspectionModeStandby()

	
	def console_joy_callback(self, msg):
		# self._print(msg)
		self.str_axis = msg.data[0]
		self.thr_axis = msg.data[1]
		self.turn_axis = msg.data[2]

		self.move_norm = np.sqrt(self.thr_axis**2 + self.str_axis**2)
		self.move_ang = np.degrees(np.arctan2(-self.str_axis, self.thr_axis))%360.0

	def height_cmd_callback(self, msg):
		self._print("height_cmd {}".format(msg.data))
		self.height_cmd = msg.data

		if self.control_mode != 3:
			if self.height_cmd != 0.0:
				self.setBodyHeight(self.height_cmd)
				self.pitch_cmd = 0.0
			else:
				self.setHomePose()
		else:
			self.got_height_cmd_inspection = True

	def pitch_cmd_callback(self, msg):
		self._print("pitch_cmd {}".format(msg.data))
		self.pitch_cmd = msg.data
		self.prev_pitch_cmd = self.pitch_cmd

		if self.control_mode != 3:
			if self.pitch_cmd != 0.0:
				self.setBodyPitch(self.pitch_cmd)
			else:
				if self.height_cmd == 0.0:
					self.setHomePose()
				else:
					self.setCustomHeightHomePose()

	def reset_height_callback(self, msg):
		self._print("reset height")
		self.height_cmd = 0.0
		self.pitch_cmd = 0.0
		self.prev_pitch_cmd = 0.0
		self.setHomePose()
		self.resetCustomHeight_homePose()

	def reset_pitch_callback(self, msg):
		self._print("reset pitch")
		self.pitch_cmd = 0.0
		self.prev_pitch_cmd = 0.0
		self.setBodyHeight(self.height_cmd)

	def leg2_xyz_cmd_callback(self, msg):
		self.got_leg2_xyz_cmd = True
		self.leg2_xyz_cmd = msg.data

	def leg3_xyz_cmd_callback(self, msg):
		self.got_leg3_xyz_cmd = True
		self.leg3_xyz_cmd = msg.data

	def inspection_start_callback(self, msg):
		self.got_inspect_start = msg.data


	#############################
	### Math helpers function ###
	#############################
	def map_with_limit(self, val, in_min, in_max, out_min, out_max):

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

	def find_ang_index(self, ang):

		ang_int = int(ang)

		ang_int_10times_less = int(ang_int)/cbw_ang_res
		before_dot = int(ang_int_10times_less)
		after_dot = (ang_int_10times_less - before_dot)*cbw_ang_res

		if after_dot <= (cbw_ang_res/2):
			ang_index = before_dot 
		else:
			ang_index = before_dot + 1

		if ang_index >= crabwalk_angle_length:
			ang_index = crabwalk_angle_length-1

		return ang_index


	###########################
	### SpiderBotLib Helper ###
	###########################
	def standByPosition(self):
		angle_list = [
						self.coxa_pre_home_ang, self.femur_pre_home_ang, self.tibia_pre_home_ang,
						self.coxa_pre_home_ang, self.femur_pre_home_ang, self.tibia_pre_home_ang,
						self.coxa_pre_home_ang, self.femur_pre_home_ang, self.tibia_pre_home_ang,
						self.coxa_pre_home_ang, self.femur_pre_home_ang, self.tibia_pre_home_ang,
						self.coxa_pre_home_ang, self.femur_pre_home_ang, self.tibia_pre_home_ang,
						self.coxa_pre_home_ang, self.femur_pre_home_ang, self.tibia_pre_home_ang,
						]
		self.setServoAngle(angle_list)
		driver.RunServoInTime(self.standby_movement_time/3, self.standby_movement_time)
		time.sleep(self.standby_movement_time/1000)
		angle_list = [
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						]
		self.setServoAngle(angle_list)
		driver.RunServoInTime(self.standby_movement_time/3, self.standby_movement_time)
		time.sleep(self.standby_movement_time/1000)

	def setServoAngle(self, angle_list):
		for i in range(len(angle_list)):
			driver.joint_deg_cmd[i+1] = angle_list[i]

	def reset_all_resetMotion_params(self):
		self.reset_seq_counter = 0
		self.home_moving = False
		self.reset_moving = False
		self.allow_check_legs_fwd = True

	def setHomePose(self):
		angle_list = [
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						self.coxa_home_ang, self.femur_home_ang, self.tibia_home_ang,
						]
		self.setServoAngle(angle_list)
		driver.RunServoInTime(self.standby_movement_time/3, self.standby_movement_time)

	def setCustomResetPose_byLeg(self, leg_no):
		"""
		When want to reset motion from custom pose (height_cmd and/or pitch_cmd not 0).

		"""

		i = leg_no - 1

		driver.joint_deg_cmd[3*i+1] = np.degrees(self.CUSTOM_HOME_LEG[leg_no]['theta1'])
		driver.joint_deg_cmd[3*i+2] = np.degrees(self.CUSTOM_HOME_LEG[leg_no]['theta2']) - 10
		driver.joint_deg_cmd[3*i+3] = np.degrees(self.CUSTOM_HOME_LEG[leg_no]['theta3'])

	def setCustomHomePose_byLeg(self, leg_no):
		"""
		Set joint_deg_cmd according to leg no. [1-6]
		self.CUSTOM_HOME_LEG[leg_no] is stored a value of the1,2,3 of custome home posture
		"""

		i = leg_no - 1

		driver.joint_deg_cmd[3*i+1] = np.degrees(self.CUSTOM_HOME_LEG[leg_no]['theta1'])
		driver.joint_deg_cmd[3*i+2] = np.degrees(self.CUSTOM_HOME_LEG[leg_no]['theta2'])
		driver.joint_deg_cmd[3*i+3] = np.degrees(self.CUSTOM_HOME_LEG[leg_no]['theta3'])



	def setCustomHeightHomePose(self):
		"""
		Set all joint_deg_cmd from stored of self.CUSTOM_HEIGHT_HOME_LEG
		self.CUSTOM_HEIGHT_HOME_LEG[leg_no] is stored a value of the1,2,3 of custom height at home posture
		"""
		
		for i in range(6):
			driver.joint_deg_cmd[3*i+1] = self.CUSTOM_HEIGHT_HOME_LEG[i+1]['theta1']
			driver.joint_deg_cmd[3*i+2] = self.CUSTOM_HEIGHT_HOME_LEG[i+1]['theta2']
			driver.joint_deg_cmd[3*i+3] = self.CUSTOM_HEIGHT_HOME_LEG[i+1]['theta3']

		driver.RunServoInTime(self.standby_movement_time/3, self.standby_movement_time)

	def resetCustomHeight_homePose(self):
		"""
		Reset the self.CUSTOM_HEIGHT_HOME_LEG back to original home from SpiderBotLib
		"""

		self.CUSTOM_HEIGHT_HOME_LEG = {
			1: {'theta1': lib.theta1_home, 'theta2': lib.theta2_home, 'theta3': lib.theta3_home,},
			2: {'theta1': lib.theta1_home, 'theta2': lib.theta2_home, 'theta3': lib.theta3_home,},
			3: {'theta1': lib.theta1_home, 'theta2': lib.theta2_home, 'theta3': lib.theta3_home,},
			4: {'theta1': lib.theta1_home, 'theta2': lib.theta2_home, 'theta3': lib.theta3_home,},
			5: {'theta1': lib.theta1_home, 'theta2': lib.theta2_home, 'theta3': lib.theta3_home,},
			6: {'theta1': lib.theta1_home, 'theta2': lib.theta2_home, 'theta3': lib.theta3_home,},
		}

	def setCrabWalkingAngle(self, ang_index, counter):
		"""
		Set all joint_deg_cmd from crab_walking_LUT_THETA
		ang_index: index value from 0->lib.cw_LUT_increment
		counter: counter number from 0 -> DATA_POINT_ALL, it will need to reset back to 0 manually
		"""

		## joint_deg_cmd index starts from 1 to 18
		## crab_walking_LUT_THETA starts from 0
		for i in range(18):
			driver.joint_deg_cmd[i+1] = np.degrees(lib.crab_walking_LUT_THETA[ang_index][i][counter])

	def setCrabWalkingAngle_custom(self, ang_index, counter):
		"""
		Set all joint_deg_cmd from crab crab_walking_LUT_custom_THETA, when height and pitch commands are given
		ang_index: index value from 0->lib.cw_LUT_increment
		counter: counter number from 0 -> DATA_POINT_ALL, it will need to reset back to 0 manually
		"""

		## joint_deg_cmd index starts from 1 to 18
		## crab_walking_LUT_custom_THETA starts from 0
		for i in range(18):
			driver.joint_deg_cmd[i+1] = np.degrees(lib.crab_walking_LUT_custom_THETA[ang_index][i][counter])

	def setTurningLeftAngle(self, counter):
		"""
		Set all joint_deg_cmd from self.TURN_CCW_LEG
		counter: counter number from 0 -> DATA_POINT_TURN_ALL
		"""

		## joint_deg_cmd index starts from 1 to 18
		## TURN_CCW_LEG has keys as leg no.
		for i in range(6):
			driver.joint_deg_cmd[3*i+1] = np.degrees(self.TURN_CCW_LEG[i+1]['theta1'][counter])
			driver.joint_deg_cmd[3*i+2] = np.degrees(self.TURN_CCW_LEG[i+1]['theta2'][counter])
			driver.joint_deg_cmd[3*i+3] = np.degrees(self.TURN_CCW_LEG[i+1]['theta3'][counter])

	def setTurningRightAngle(self, counter):
		"""
		Set all joint_deg_cmd from self.TURN_CW_LEG
		counter: counter number from 0 -> DATA_POINT_TURN_ALL
		"""

		## joint_deg_cmd index starts from 1 to 18
		## TURN_CW_LEG has keys as leg no.
		for i in range(6):
			driver.joint_deg_cmd[3*i+1] = np.degrees(self.TURN_CW_LEG[i+1]['theta1'][counter])
			driver.joint_deg_cmd[3*i+2] = np.degrees(self.TURN_CW_LEG[i+1]['theta2'][counter])
			driver.joint_deg_cmd[3*i+3] = np.degrees(self.TURN_CW_LEG[i+1]['theta3'][counter])

	def setCustomTurningLeftLegAngle(self, counter):
		"""
		Set all joint_deg_cmd from self.TURN_CUSTOM_CCW_LEG, using when height and/or pitch commands are given
		counter: counter number from 0 -> DATA_POINT_TURN_ALL
		"""

		## joint_deg_cmd index starts from 1 to 18
		## TURN_CW_LEG has keys as leg no.
		for i in range(6):
			driver.joint_deg_cmd[3*i+1] = np.degrees(self.TURN_CUSTOM_CCW_LEG[i+1]['theta1'][counter])
			driver.joint_deg_cmd[3*i+2] = np.degrees(self.TURN_CUSTOM_CCW_LEG[i+1]['theta2'][counter])
			driver.joint_deg_cmd[3*i+3] = np.degrees(self.TURN_CUSTOM_CCW_LEG[i+1]['theta3'][counter])

	def setCustomTurningRightLegAngle(self, counter):
		"""
		Set all joint_deg_cmd from self.TURN_CUSTOM_CW_LEG, using when height and/or pitch commands are given
		counter: counter number from 0 -> DATA_POINT_TURN_ALL
		"""

		## joint_deg_cmd index starts from 1 to 18
		## TURN_CW_LEG has keys as leg no.
		for i in range(6):
			driver.joint_deg_cmd[3*i+1] = np.degrees(self.TURN_CUSTOM_CW_LEG[i+1]['theta1'][counter])
			driver.joint_deg_cmd[3*i+2] = np.degrees(self.TURN_CUSTOM_CW_LEG[i+1]['theta2'][counter])
			driver.joint_deg_cmd[3*i+3] = np.degrees(self.TURN_CUSTOM_CW_LEG[i+1]['theta3'][counter])

	def setBodyControlRotationAngle(self, R, P, Y):
		"""
		Input roll-pitch-yaw angle from joystick and use lib.bodyRotate_to_newLegXYZ and lib.inv
		to find the angle of each joint
		Then set all joint_deg_cmd according to joints output
		"""

		r = np.radians(R)
		p = np.radians(P)
		y = np.radians(Y)
		leg_XYZ  = lib.bodyRotate_to_newLegXYZ(r,p,y)

		leg_i_XYZ = leg_XYZ[0]
		leg_j_XYZ = leg_XYZ[1]
		leg_k_XYZ = leg_XYZ[2]
		leg_l_XYZ = leg_XYZ[3]
		leg_m_XYZ = leg_XYZ[4]
		leg_n_XYZ = leg_XYZ[5]

		LEG = {
			1: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
			2: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
			3: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
			4: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
			5: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
			6: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
		}

		LEG[1]['theta1'], LEG[1]['theta2'], LEG[1]['theta3'] = lib.inv(leg_i_XYZ[0], leg_i_XYZ[1], leg_i_XYZ[2])
		LEG[2]['theta1'], LEG[2]['theta2'], LEG[2]['theta3'] = lib.inv(leg_j_XYZ[0], leg_j_XYZ[1], leg_j_XYZ[2])
		LEG[3]['theta1'], LEG[3]['theta2'], LEG[3]['theta3'] = lib.inv(leg_k_XYZ[0], leg_k_XYZ[1], leg_k_XYZ[2])
		LEG[4]['theta1'], LEG[4]['theta2'], LEG[4]['theta3'] = lib.inv(leg_l_XYZ[0], leg_l_XYZ[1], leg_l_XYZ[2])
		LEG[5]['theta1'], LEG[5]['theta2'], LEG[5]['theta3'] = lib.inv(leg_m_XYZ[0], leg_m_XYZ[1], leg_m_XYZ[2])
		LEG[6]['theta1'], LEG[6]['theta2'], LEG[6]['theta3'] = lib.inv(leg_n_XYZ[0], leg_n_XYZ[1], leg_n_XYZ[2])

		for i in range(6):
			driver.joint_deg_cmd[3*i+1] = np.degrees(LEG[i+1]['theta1'])
			driver.joint_deg_cmd[3*i+2] = np.degrees(LEG[i+1]['theta2'])
			driver.joint_deg_cmd[3*i+3] = np.degrees(LEG[i+1]['theta3'])


	def setBodyControlRotationAngle_CustomHeight(self, R, P, Y):
		"""
		Input roll-pitch-yaw angle from joystick and use lib.bodyRotate_to_newLegXYZ_customHome and lib.inv
		to find the angle of each joint, and the start height could be self.customHeight_home_xyz[2]
		Then set all joint_deg_cmd according to joints output
		"""

		r = np.radians(R)
		p = np.radians(P)
		y = np.radians(Y)
		leg_XYZ  = lib.bodyRotate_to_newLegXYZ_customHome(r,p,y, self.customHeight_home_xyz[2])

		leg_i_XYZ = leg_XYZ[0]
		leg_j_XYZ = leg_XYZ[1]
		leg_k_XYZ = leg_XYZ[2]
		leg_l_XYZ = leg_XYZ[3]
		leg_m_XYZ = leg_XYZ[4]
		leg_n_XYZ = leg_XYZ[5]

		LEG = {
			1: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
			2: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
			3: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
			4: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
			5: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
			6: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
		}

		LEG[1]['theta1'], LEG[1]['theta2'], LEG[1]['theta3'] = lib.inv(leg_i_XYZ[0], leg_i_XYZ[1], leg_i_XYZ[2])
		LEG[2]['theta1'], LEG[2]['theta2'], LEG[2]['theta3'] = lib.inv(leg_j_XYZ[0], leg_j_XYZ[1], leg_j_XYZ[2])
		LEG[3]['theta1'], LEG[3]['theta2'], LEG[3]['theta3'] = lib.inv(leg_k_XYZ[0], leg_k_XYZ[1], leg_k_XYZ[2])
		LEG[4]['theta1'], LEG[4]['theta2'], LEG[4]['theta3'] = lib.inv(leg_l_XYZ[0], leg_l_XYZ[1], leg_l_XYZ[2])
		LEG[5]['theta1'], LEG[5]['theta2'], LEG[5]['theta3'] = lib.inv(leg_m_XYZ[0], leg_m_XYZ[1], leg_m_XYZ[2])
		LEG[6]['theta1'], LEG[6]['theta2'], LEG[6]['theta3'] = lib.inv(leg_n_XYZ[0], leg_n_XYZ[1], leg_n_XYZ[2])

		for i in range(6):
			driver.joint_deg_cmd[3*i+1] = np.degrees(LEG[i+1]['theta1'])
			driver.joint_deg_cmd[3*i+2] = np.degrees(LEG[i+1]['theta2'])
			driver.joint_deg_cmd[3*i+3] = np.degrees(LEG[i+1]['theta3'])

	def setBodyHeight(self, height):
		"""
		Set the body height according to height command by using lib.bodyTranslate_to_newLegXYZ and lib.inv
		height: the default start value is from 0 (body frame), then adjust up-down in mm
		Then set all joint_deg_cmd according to 
		"""
		PC_new = np.array([0.0, 0.0, height])
		leg_XYZ  = lib.bodyTranslate_to_newLegXYZ(PC_new)
		leg_i_XYZ = leg_XYZ[0]
		leg_j_XYZ = leg_XYZ[1]
		leg_k_XYZ = leg_XYZ[2]
		leg_l_XYZ = leg_XYZ[3]
		leg_m_XYZ = leg_XYZ[4]
		leg_n_XYZ = leg_XYZ[5]

		self.customHeight_home_xyz = [leg_i_XYZ[0], leg_i_XYZ[1], leg_i_XYZ[2]]

		LEG = {
			1: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
			2: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
			3: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
			4: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
			5: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
			6: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
		}

		LEG[1]['theta1'], LEG[1]['theta2'], LEG[1]['theta3'] = lib.inv(leg_i_XYZ[0], leg_i_XYZ[1], leg_i_XYZ[2])
		LEG[2]['theta1'], LEG[2]['theta2'], LEG[2]['theta3'] = lib.inv(leg_j_XYZ[0], leg_j_XYZ[1], leg_j_XYZ[2])
		LEG[3]['theta1'], LEG[3]['theta2'], LEG[3]['theta3'] = lib.inv(leg_k_XYZ[0], leg_k_XYZ[1], leg_k_XYZ[2])
		LEG[4]['theta1'], LEG[4]['theta2'], LEG[4]['theta3'] = lib.inv(leg_l_XYZ[0], leg_l_XYZ[1], leg_l_XYZ[2])
		LEG[5]['theta1'], LEG[5]['theta2'], LEG[5]['theta3'] = lib.inv(leg_m_XYZ[0], leg_m_XYZ[1], leg_m_XYZ[2])
		LEG[6]['theta1'], LEG[6]['theta2'], LEG[6]['theta3'] = lib.inv(leg_n_XYZ[0], leg_n_XYZ[1], leg_n_XYZ[2])

		for i in range(6):
			driver.joint_deg_cmd[3*i+1] = np.degrees(LEG[i+1]['theta1'])
			driver.joint_deg_cmd[3*i+2] = np.degrees(LEG[i+1]['theta2'])
			driver.joint_deg_cmd[3*i+3] = np.degrees(LEG[i+1]['theta3'])

			self.CUSTOM_HOME_LEG[i+1]['theta1'] = LEG[i+1]['theta1']
			self.CUSTOM_HOME_LEG[i+1]['theta2'] = LEG[i+1]['theta2']
			self.CUSTOM_HOME_LEG[i+1]['theta3'] = LEG[i+1]['theta3']

			self.CUSTOM_HEIGHT_HOME_LEG[i+1]['theta1'] = LEG[i+1]['theta1']
			self.CUSTOM_HEIGHT_HOME_LEG[i+1]['theta2'] = LEG[i+1]['theta2']
			self.CUSTOM_HEIGHT_HOME_LEG[i+1]['theta3'] = LEG[i+1]['theta3']

		driver.RunServoInTime(self.height_pitch_cmd_time/2, self.height_pitch_cmd_time)

		if self.control_mode != 2:
			lib.generate_crabWalkingLUT_custom(0.0, leg_XYZ)
			self.generateInPlaceTurningCustom()

	def generateInPlaceTurningCustom(self):
		"""
		When there is height_cmd or pitch_cmd coming, the turning motion will be diffent that home position.
		It needs to use customHeight_hom_xyz as reference height.
		This is to calculate TURN_CUSTOM_CCW_LEG and TURN_CUSTOM_CW_LEG
		"""

		legIKM_CCW_XYZ, legJLN_CCW_XYZ = lib.generate_inplace_turning_customHeight(-self.turning_angle, self.customHeight_home_xyz)
		self.TURN_CUSTOM_CCW_LEG[1]['theta1'], self.TURN_CUSTOM_CCW_LEG[1]['theta2'], self.TURN_CUSTOM_CCW_LEG[1]['theta3'] = lib.invKinArray_to_ThetaArray(legIKM_CCW_XYZ)
		self.TURN_CUSTOM_CCW_LEG[3]['theta1'], self.TURN_CUSTOM_CCW_LEG[3]['theta2'], self.TURN_CUSTOM_CCW_LEG[3]['theta3'] = self.TURN_CUSTOM_CCW_LEG[1]['theta1'], self.TURN_CUSTOM_CCW_LEG[1]['theta2'], self.TURN_CUSTOM_CCW_LEG[1]['theta3']
		self.TURN_CUSTOM_CCW_LEG[5]['theta1'], self.TURN_CUSTOM_CCW_LEG[5]['theta2'], self.TURN_CUSTOM_CCW_LEG[5]['theta3'] = self.TURN_CUSTOM_CCW_LEG[1]['theta1'], self.TURN_CUSTOM_CCW_LEG[1]['theta2'], self.TURN_CUSTOM_CCW_LEG[1]['theta3']
		self.TURN_CUSTOM_CCW_LEG[2]['theta1'], self.TURN_CUSTOM_CCW_LEG[2]['theta2'], self.TURN_CUSTOM_CCW_LEG[2]['theta3'] = lib.invKinArray_to_ThetaArray(legJLN_CCW_XYZ)
		self.TURN_CUSTOM_CCW_LEG[4]['theta1'], self.TURN_CUSTOM_CCW_LEG[4]['theta2'], self.TURN_CUSTOM_CCW_LEG[4]['theta3'] = self.TURN_CUSTOM_CCW_LEG[2]['theta1'], self.TURN_CUSTOM_CCW_LEG[2]['theta2'], self.TURN_CUSTOM_CCW_LEG[2]['theta3']
		self.TURN_CUSTOM_CCW_LEG[6]['theta1'], self.TURN_CUSTOM_CCW_LEG[6]['theta2'], self.TURN_CUSTOM_CCW_LEG[6]['theta3'] = self.TURN_CUSTOM_CCW_LEG[2]['theta1'], self.TURN_CUSTOM_CCW_LEG[2]['theta2'], self.TURN_CUSTOM_CCW_LEG[2]['theta3']

		legIKM_CW_XYZ, legJLN_CW_XYZ = lib.generate_inplace_turning_customHeight(self.turning_angle, self.customHeight_home_xyz)
		self.TURN_CUSTOM_CW_LEG[1]['theta1'], self.TURN_CUSTOM_CW_LEG[1]['theta2'], self.TURN_CUSTOM_CW_LEG[1]['theta3'] = lib.invKinArray_to_ThetaArray(legIKM_CW_XYZ)
		self.TURN_CUSTOM_CW_LEG[3]['theta1'], self.TURN_CUSTOM_CW_LEG[3]['theta2'], self.TURN_CUSTOM_CW_LEG[3]['theta3'] = self.TURN_CUSTOM_CW_LEG[1]['theta1'], self.TURN_CUSTOM_CW_LEG[1]['theta2'], self.TURN_CUSTOM_CW_LEG[1]['theta3']
		self.TURN_CUSTOM_CW_LEG[5]['theta1'], self.TURN_CUSTOM_CW_LEG[5]['theta2'], self.TURN_CUSTOM_CW_LEG[5]['theta3'] = self.TURN_CUSTOM_CW_LEG[1]['theta1'], self.TURN_CUSTOM_CW_LEG[1]['theta2'], self.TURN_CUSTOM_CW_LEG[1]['theta3']
		self.TURN_CUSTOM_CW_LEG[2]['theta1'], self.TURN_CUSTOM_CW_LEG[2]['theta2'], self.TURN_CUSTOM_CW_LEG[2]['theta3'] = lib.invKinArray_to_ThetaArray(legJLN_CW_XYZ)
		self.TURN_CUSTOM_CW_LEG[4]['theta1'], self.TURN_CUSTOM_CW_LEG[4]['theta2'], self.TURN_CUSTOM_CW_LEG[4]['theta3'] = self.TURN_CUSTOM_CW_LEG[2]['theta1'], self.TURN_CUSTOM_CW_LEG[2]['theta2'], self.TURN_CUSTOM_CW_LEG[2]['theta3'] 
		self.TURN_CUSTOM_CW_LEG[6]['theta1'], self.TURN_CUSTOM_CW_LEG[6]['theta2'], self.TURN_CUSTOM_CW_LEG[6]['theta3'] = self.TURN_CUSTOM_CW_LEG[2]['theta1'], self.TURN_CUSTOM_CW_LEG[2]['theta2'], self.TURN_CUSTOM_CW_LEG[2]['theta3'] 


	def setBodyPitch(self, P):
		"""
		Set the body pitch according to pitch command by using lib.bodyRotate_to_newLegXYZ_customHome and lib.inv
		P: is the pitch angle in degree.
		setBodyPitch has to setup from flat postion, meaning once setBodyHeight or from home this one can be called. 
		If change height after this, the pitch command will reset and we need to set it again on that new height.
		"""

		r = np.radians(P)
		leg_XYZ  = lib.bodyRotate_to_newLegXYZ_customHome(r, 0.0, 0.0, lib.Z_home-self.height_cmd)

		leg_i_XYZ = leg_XYZ[0]
		leg_j_XYZ = leg_XYZ[1]
		leg_k_XYZ = leg_XYZ[2]
		leg_l_XYZ = leg_XYZ[3]
		leg_m_XYZ = leg_XYZ[4]
		leg_n_XYZ = leg_XYZ[5]

		LEG = {
			1: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
			2: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
			3: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
			4: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
			5: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
			6: {'theta1': 0.0, 'theta2': 0.0, 'theta3': 0.0,},
		}

		LEG[1]['theta1'], LEG[1]['theta2'], LEG[1]['theta3'] = lib.inv(leg_i_XYZ[0], leg_i_XYZ[1], leg_i_XYZ[2])
		LEG[2]['theta1'], LEG[2]['theta2'], LEG[2]['theta3'] = lib.inv(leg_j_XYZ[0], leg_j_XYZ[1], leg_j_XYZ[2])
		LEG[3]['theta1'], LEG[3]['theta2'], LEG[3]['theta3'] = lib.inv(leg_k_XYZ[0], leg_k_XYZ[1], leg_k_XYZ[2])
		LEG[4]['theta1'], LEG[4]['theta2'], LEG[4]['theta3'] = lib.inv(leg_l_XYZ[0], leg_l_XYZ[1], leg_l_XYZ[2])
		LEG[5]['theta1'], LEG[5]['theta2'], LEG[5]['theta3'] = lib.inv(leg_m_XYZ[0], leg_m_XYZ[1], leg_m_XYZ[2])
		LEG[6]['theta1'], LEG[6]['theta2'], LEG[6]['theta3'] = lib.inv(leg_n_XYZ[0], leg_n_XYZ[1], leg_n_XYZ[2])

		for i in range(6):
			driver.joint_deg_cmd[3*i+1] = np.degrees(LEG[i+1]['theta1'])
			driver.joint_deg_cmd[3*i+2] = np.degrees(LEG[i+1]['theta2'])
			driver.joint_deg_cmd[3*i+3] = np.degrees(LEG[i+1]['theta3'])

			self.CUSTOM_HOME_LEG[i+1]['theta1'] = LEG[i+1]['theta1']
			self.CUSTOM_HOME_LEG[i+1]['theta2'] = LEG[i+1]['theta2']
			self.CUSTOM_HOME_LEG[i+1]['theta3'] = LEG[i+1]['theta3']

		driver.RunServoInTime(self.height_pitch_cmd_time/2, self.height_pitch_cmd_time)
		lib.generate_crabWalkingLUT_custom(r, leg_XYZ)

	def setInspectionModeStandby(self):

		stanby_base_ang = 30
		lift_ang = 20
		inspection_standby_time = 100

		## lift up leg1 a bit
		driver.joint_deg_cmd[1] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[1]['theta1']) + stanby_base_ang
		driver.joint_deg_cmd[2] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[1]['theta2']) + lift_ang
		driver.joint_deg_cmd[3] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[1]['theta3'])
		driver.RunServoInTimeByLeg(acc_time=inspection_standby_time//2, finish_time=inspection_standby_time, leg_no=1)
		time.sleep(inspection_standby_time/1000)
		## lift down leg1 on standby
		driver.joint_deg_cmd[2] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[1]['theta2'])
		driver.RunServoInTimeByLeg(acc_time=inspection_standby_time//2, finish_time=inspection_standby_time, leg_no=1)
		time.sleep(inspection_standby_time/1000)

		# driver.joint_deg_cmd[4] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[2]['theta1'])
		# driver.joint_deg_cmd[5] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[2]['theta2'])
		# driver.joint_deg_cmd[6] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[2]['theta3'])

		# driver.joint_deg_cmd[7] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[3]['theta1'])
		# driver.joint_deg_cmd[8] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[3]['theta2'])
		# driver.joint_deg_cmd[9] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[3]['theta3'])

		## lift up leg4 a bit
		driver.joint_deg_cmd[10] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[4]['theta1']) - stanby_base_ang
		driver.joint_deg_cmd[11] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[4]['theta2']) + lift_ang
		driver.joint_deg_cmd[12] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[4]['theta3'])
		driver.RunServoInTimeByLeg(acc_time=inspection_standby_time//2, finish_time=inspection_standby_time, leg_no=4)
		time.sleep(inspection_standby_time/1000)
		## lift down leg4 on standby
		driver.joint_deg_cmd[11] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[4]['theta2'])
		driver.RunServoInTimeByLeg(acc_time=inspection_standby_time//2, finish_time=inspection_standby_time, leg_no=4)
		time.sleep(inspection_standby_time/1000)

		## lift up leg5 a bit
		driver.joint_deg_cmd[13] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[5]['theta1']) + stanby_base_ang
		driver.joint_deg_cmd[14] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[5]['theta2']) + lift_ang
		driver.joint_deg_cmd[15] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[5]['theta3'])
		driver.RunServoInTimeByLeg(acc_time=inspection_standby_time//2, finish_time=inspection_standby_time, leg_no=5)
		time.sleep(inspection_standby_time/1000)
		## lift down leg5 on standby
		driver.joint_deg_cmd[14] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[5]['theta2'])
		driver.RunServoInTimeByLeg(acc_time=inspection_standby_time//2, finish_time=inspection_standby_time, leg_no=5)
		time.sleep(inspection_standby_time/1000)

		## lift up leg6 a bit
		driver.joint_deg_cmd[16] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[6]['theta1']) - stanby_base_ang
		driver.joint_deg_cmd[17] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[6]['theta2']) + lift_ang
		driver.joint_deg_cmd[18] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[6]['theta3'])
		driver.RunServoInTimeByLeg(acc_time=inspection_standby_time//2, finish_time=inspection_standby_time, leg_no=6)
		time.sleep(inspection_standby_time/1000)
		## lift down leg6 on standby
		driver.joint_deg_cmd[17] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[6]['theta2'])
		driver.RunServoInTimeByLeg(acc_time=inspection_standby_time//2, finish_time=inspection_standby_time, leg_no=6)
		time.sleep(inspection_standby_time/1000)

		## leg2
		driver.joint_deg_cmd[4] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[2]['theta1']) + 40
		driver.joint_deg_cmd[5] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[2]['theta2']) + 45
		driver.joint_deg_cmd[6] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[2]['theta3'])
		## leg3
		driver.joint_deg_cmd[7] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[3]['theta1']) - 40
		driver.joint_deg_cmd[8] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[3]['theta2']) + 45
		driver.joint_deg_cmd[9] = np.degrees(self.CUSTOM_HEIGHT_HOME_LEG[3]['theta3'])

		driver.RunServoInTimeByLeg(acc_time=self.standby_movement_time//2, finish_time=self.standby_movement_time, leg_no=2)
		driver.RunServoInTimeByLeg(acc_time=self.standby_movement_time//2, finish_time=self.standby_movement_time, leg_no=3)
		time.sleep(self.standby_movement_time/1000)

		leg1_x, leg1_y, leg1_z = lib.fwd(np.radians(driver.joint_deg_cmd[1]), np.radians(driver.joint_deg_cmd[2]), np.radians(driver.joint_deg_cmd[3]))
		self.leg2_x, self.leg2_y, self.leg2_z = lib.fwd(np.radians(driver.joint_deg_cmd[4]), np.radians(driver.joint_deg_cmd[5]), np.radians(driver.joint_deg_cmd[6]))
		self.leg3_x, self.leg3_y, self.leg3_z = lib.fwd(np.radians(driver.joint_deg_cmd[7]), np.radians(driver.joint_deg_cmd[8]), np.radians(driver.joint_deg_cmd[9]))
		leg4_x, leg4_y, leg4_z = lib.fwd(np.radians(driver.joint_deg_cmd[10]), np.radians(driver.joint_deg_cmd[11]), np.radians(driver.joint_deg_cmd[12]))
		leg5_x, leg5_y, leg5_z = lib.fwd(np.radians(driver.joint_deg_cmd[13]), np.radians(driver.joint_deg_cmd[14]), np.radians(driver.joint_deg_cmd[15]))
		leg6_x, leg6_y, leg6_z = lib.fwd(np.radians(driver.joint_deg_cmd[16]), np.radians(driver.joint_deg_cmd[17]), np.radians(driver.joint_deg_cmd[18]))

		# self._print(leg1_z, leg4_z, leg5_z, leg6_z)
		self.INSPECT_Z_Home = (leg1_z + leg4_z + leg5_z + leg6_z) / 4
		# self._print("INSPECT_Z_Home", self.INSPECT_Z_Home)

		self.insp_leg_xyz = {'x': self.leg2_x, 'y': self.leg2_y, 'z': self.leg2_z}

		## set new custom home leg
		# for i in range(6):
		# 	leg_no = i + 1
		# 	self.CUSTOM_HOME_LEG[leg_no]['theta1'] = np.radians(driver.joint_deg_cmd[3*i+1])
		# 	self.CUSTOM_HOME_LEG[leg_no]['theta2'] = np.radians(driver.joint_deg_cmd[3*i+2])
		# 	self.CUSTOM_HOME_LEG[leg_no]['theta3'] = np.radians(driver.joint_deg_cmd[3*i+3])

	def resetMotion_toHome(self):
		reset_done = False
		while not reset_done:
			### firstly check which leg is lifting highest, then will be moving that one first
			if self.allow_check_legs_fwd:
				self.allow_check_legs_fwd = False
				self.reset_leg_seq_list = []
				
				driver.ReadPosition() ## it tooks 0.03 on Jetson Nano
				
				leg1_x, leg1_y, leg1_z = lib.fwd(np.radians(driver.joint_position[1]) , np.radians(driver.joint_position[2]) , np.radians(driver.joint_position[3]))
				leg2_x, leg2_y, leg2_z = lib.fwd(np.radians(driver.joint_position[4]) , np.radians(driver.joint_position[5]) , np.radians(driver.joint_position[6]))
				leg3_x, leg3_y, leg3_z = lib.fwd(np.radians(driver.joint_position[7]) , np.radians(driver.joint_position[8]) , np.radians(driver.joint_position[9]))
				leg4_x, leg4_y, leg4_z = lib.fwd(np.radians(driver.joint_position[10]), np.radians(driver.joint_position[11]), np.radians(driver.joint_position[12]))
				leg5_x, leg5_y, leg5_z = lib.fwd(np.radians(driver.joint_position[13]), np.radians(driver.joint_position[14]), np.radians(driver.joint_position[15]))
				leg6_x, leg6_y, leg6_z = lib.fwd(np.radians(driver.joint_position[16]), np.radians(driver.joint_position[17]), np.radians(driver.joint_position[18]))

				z_array = np.array([leg1_z, leg2_z, leg3_z, leg4_z, leg5_z, leg6_z])
				for i in range(6):
					most_lift_up_idx = np.argmax(z_array)
					self.reset_leg_seq_list.append(most_lift_up_idx+1)
					z_array[most_lift_up_idx] = -1000

				self._print("reset_leg_seq_list {}".format(self.reset_leg_seq_list))
				# quit()

			if (not self.reset_moving) and (not self.home_moving):

				self.leg_no = self.reset_leg_seq_list[self.reset_seq_counter]

				if self.leg_no == 1:
					self.start_id = 1

				elif self.leg_no == 2:
					self.start_id = 4

				elif self.leg_no == 3:
					self.start_id = 7

				elif self.leg_no == 4:
					self.start_id = 10

				elif self.leg_no == 5:
					self.start_id = 13

				elif self.leg_no == 6:
					self.start_id = 16

				self._print("reset update leg number {:d}".format(self.leg_no))

				self.reset_moving = True

				if (self.height_cmd == 0.0) and (self.pitch_cmd == 0.0):
					if self.prev_pitch_cmd == 0.0:
						driver.joint_deg_cmd[self.start_id] = np.degrees(self.theta1_reset)
						driver.joint_deg_cmd[self.start_id+1] = np.degrees(self.theta2_reset)
						driver.joint_deg_cmd[self.start_id+2] = np.degrees(self.theta3_reset)
						self._print("do reset pose")
					else:
						self.setCustomResetPose_byLeg(self.leg_no)
						self._print("do reset pose inside")

				else:
					self.setCustomResetPose_byLeg(self.leg_no)
					self._print("do reset custom pose")

				driver.RunServoInTimeByLeg(acc_time=self.reset_movement_time//2, finish_time=self.reset_movement_time, leg_no=self.leg_no)
				self.last_reset_movement_stamp = time.time()
				

			### After reset movement is done, then move to home
			if ((time.time() - self.last_reset_movement_stamp) > self.reset_movement_time/1000) and self.reset_moving:
				self.home_moving = True
				self.reset_moving = False

				if (self.height_cmd == 0.0) and (self.pitch_cmd == 0.0):
					if self.prev_pitch_cmd == 0.0:
						driver.joint_deg_cmd[self.start_id] = self.coxa_home_ang
						driver.joint_deg_cmd[self.start_id+1] = self.femur_home_ang
						driver.joint_deg_cmd[self.start_id+2] = self.tibia_home_ang
						self._print("do home pose")
					else:
						self.setCustomHomePose_byLeg(self.leg_no)
						self._print("do custom home pose inside")

				else:
					self.setCustomHomePose_byLeg(self.leg_no)
					self._print("do custom home pose")

				driver.RunServoInTimeByLeg(acc_time=self.reset_movement_time//2, finish_time=self.reset_movement_time, leg_no=self.leg_no)

				self.last_home_movement_stamp = time.time()



			if ((time.time() - self.last_home_movement_stamp) > self.reset_movement_time/1000) and self.home_moving:
				self.home_moving = False
				self.reset_moving = False
				self.reset_seq_counter += 1

			## Lastly check if counter is over than 6 means done reseting all legs
			if self.reset_seq_counter >= 6:
				self._print("Done all reset")
				self.allow_reset = False
				self.reset_seq_counter = 0
				self.home_moving = False
				self.reset_moving = False
				self.allow_check_legs_fwd = True

				self.walk_counter = 0
				self.turn_counter = 0
				self.wg_walk_counter = 0

				self.pitch_cmd = self.prev_pitch_cmd
				self.inspect_standby = False
				reset_done = True

	def readCurrentAndPosition(self):
		driver.ReadCurrentPosition()
		self.leg2_x_fb, self.leg2_y_fb, self.leg2_z_fb = lib.fwd(np.radians(driver.joint_cur_pos[4]['pos']) , np.radians(driver.joint_cur_pos[5]['pos']) , np.radians(driver.joint_cur_pos[6]['pos']))
		self.leg3_x_fb, self.leg3_y_fb, self.leg3_z_fb = lib.fwd(np.radians(driver.joint_cur_pos[7]['pos']) , np.radians(driver.joint_cur_pos[8]['pos']) , np.radians(driver.joint_cur_pos[9]['pos']))

		self.servo_cur_fb = {
			5: driver.joint_cur_pos[5]['cur'],
			6: driver.joint_cur_pos[6]['cur'],
			8: driver.joint_cur_pos[8]['cur'],
			9: driver.joint_cur_pos[9]['cur'],
		}

	def pub_insp_status(self, status_num):
		status_msg  = Int8()
		status_msg.data = status_num
		self.inspect_status_pub.publish(status_msg)


	############
	### Loop ###
	############
	def timer_callback(self):

	
		################################
		### RC transmitter operation ###
		################################
		if self.cart_mode == 1:

			################################
			### Crab-walking and Turning ###
			################################
			if self.movement_mode == "crabwalk":

				## Throttle stick is forward
				if self.sbus_thr >= self.sbus_max_db:

					### Mapping throttle to delay_cb_time
					self.delay_cb_time = int(self.map_with_limit(self.sbus_thr, self.sbus_max_db, self.sbus_max, self.biggest_delay, self.smallest_delay))

					### Mapping steering to ang_index half-front
					if self.sbus_str >= self.sbus_max_db:
							self.ang_index = int(self.map_with_limit(self.sbus_str, self.sbus_max_db, self.sbus_max, self.cw_LUT_last_idx, self.full_right_idx))
					elif self.sbus_str <= self.sbus_min_db:
							self.ang_index = int(self.map_with_limit(self.sbus_str, self.sbus_min, self.sbus_min_db, self.full_left_idx, 0))
					else:
						self.ang_index = 0

					self.robot_mode = "CRAB"

				## Throttle stick is backward
				elif self.sbus_thr <= self.sbus_min_db:
					### Mapping throttle to delay_cb_time
					self.delay_cb_time = int(self.map_with_limit(self.sbus_thr, self.sbus_min, self.sbus_min_db, self.smallest_delay, self.biggest_delay))

					### Mapping steering to ang_index half-back
					if self.sbus_str >= self.sbus_max_db:
							self.ang_index = int(self.map_with_limit(self.sbus_str, self.sbus_max_db, self.sbus_max, self.full_back_idx, self.full_right_idx+1))
					elif self.sbus_str <= self.sbus_min_db:
							self.ang_index = int(self.map_with_limit(self.sbus_str, self.sbus_min, self.sbus_min_db, self.full_left_idx, self.full_back_idx))
					else:
						self.ang_index = self.full_back_idx

					self.robot_mode = "CRAB"

				## Throttle is in middle, steering is pushed
				elif (self.sbus_min_db < self.sbus_thr < self.sbus_max_db) and ((self.sbus_str <= self.sbus_min_db) or (self.sbus_str >= self.sbus_max_db)):
					
					if self.sbus_str <= self.sbus_min_db:
						self.delay_turning_time = int(self.map_with_limit(self.sbus_str, self.sbus_min, self.sbus_min_db, self.smallest_delay, self.biggest_delay))
					elif self.sbus_str >= self.sbus_max_db:
						self.delay_turning_time = int(self.map_with_limit(self.sbus_str, self.sbus_max_db, self.sbus_max, self.biggest_delay, self.smallest_delay))
					else:
						self.delay_turning_time = self.biggest_delay

					self.robot_mode = "TURN"

				else:
					self.delay_cb_time = self.biggest_delay
					self.ang_index = 0
					self.robot_mode = "STOP"


				### Decide what to do ###
				cond_for_crabWalking = ((time.time() - self.last_crab_walk_stamp) >= (self.delay_cb_time/1000)) and \
										((self.sbus_thr <= self.sbus_min_db) or (self.sbus_thr >= self.sbus_max_db))

				cond_for_turning = ((time.time() - self.last_turn_stamp) >= (self.delay_turning_time/1000)) and \
									(self.sbus_min_db <  self.sbus_thr < self.sbus_max_db) and \
									((self.sbus_str <= self.sbus_min_db) or (self.sbus_str >= self.sbus_max_db))

				if cond_for_crabWalking:

					if (self.walk_counter < DATA_POINT_ALL):
						self.setCrabWalkingAngle(self.ang_index, self.walk_counter)
						self.walk_counter += 1
						# start_time = time.time()
						driver.RunServoInTime(acc_time=0, finish_time=self.delay_cb_time)
						# period = time.time() - start_time
						# self._print("period", period)

						self.last_crab_walk_stamp = time.time()

					## reset walk_counter
					if (self.walk_counter >= DATA_POINT_ALL):
						self.walk_counter = 0

					self.allow_reset = True
					self.reset_all_resetMotion_params()

				elif cond_for_turning:

					if (self.turn_counter < DATA_POINT_TURN_ALL):
						if self.sbus_str <= self.sbus_min_db:
							self.setTurningLeftAngle(self.turn_counter)
						elif self.sbus_str >= self.sbus_max_db:
							self.setTurningRightAngle(self.turn_counter)
						
						self.turn_counter += 1
						driver.RunServoInTime(acc_time=0, finish_time=self.delay_turning_time)
						self.last_turn_stamp = time.time()

					## reset turn counter
					if (self.turn_counter >= DATA_POINT_TURN_ALL):
						self.turn_counter = 0

					self.allow_reset = True
					self.reset_all_resetMotion_params()

			#############################
			### Body Rotation Control ###
			#############################
			elif self.movement_mode == "bodycontrol":

				self.roll_cmd_deg = self.map_with_limit(self.sbus_roll, self.sbus_min, self.sbus_max, -self.max_body_rotate_deg, self.max_body_rotate_deg)
				self.pitch_cmd_deg = self.map_with_limit(self.sbus_pitch, self.sbus_min, self.sbus_max, -self.max_body_rotate_deg, self.max_body_rotate_deg)
				self.yaw_cmd_deg = self.map_with_limit(self.sbus_yaw, self.sbus_min, self.sbus_max, self.max_body_rotate_deg, -self.max_body_rotate_deg)

				self.setBodyControlRotationAngle(self.roll_cmd_deg, self.pitch_cmd_deg, self.yaw_cmd_deg)

				if (time.time() - self.last_body_rotation_stamp) > self.delay_body_rotaiton/1000:
					driver.RunServoInTime(acc_time=0, finish_time=self.delay_body_rotaiton)
					self.last_body_rotation_stamp = time.time()

				self.robot_mode = "BROT"

				self.allow_reset = True
				self.reset_all_resetMotion_params()

		#############################
		### Web console operation ###
		#############################
		elif self.cart_mode == 2:

			####################
			### Flat-walking ###
			####################
			if self.control_mode == 0:

				if (abs(self.thr_axis) > 0.0) or (abs(self.str_axis) > 0):

					self.delay_cb_time = self.map_with_limit(self.move_norm, 0.0, 1.0, self.biggest_delay, self.smallest_delay)
					self.ang_index = self.find_ang_index(self.move_ang)

					### body is at home, normal flat-walking ###
					if (self.height_cmd == 0.0) and (self.pitch_cmd == 0.0):
						if ((time.time() - self.last_crab_walk_stamp) >= self.delay_cb_time/1000) and ((self.move_norm > 0.1)):

							if (self.walk_counter < DATA_POINT_ALL):
								self.setCrabWalkingAngle(self.ang_index, self.walk_counter)
								self.walk_counter += 1
								self.last_crab_walk_stamp = time.time()
								driver.RunServoInTime(acc_time=0, finish_time=self.delay_cb_time)
								

							if (self.walk_counter >= DATA_POINT_ALL):
								self.walk_counter = 0

							self.allow_reset = True
							

					### body was adjusted by height or pitch cmds ###
					else:

						if ((time.time() - self.last_crab_walk_custom_stamp) >= self.delay_cb_time/1000):

							if (self.walk_custom_counter < DATA_POINT_ALL):
								self.setCrabWalkingAngle_custom(self.ang_index, self.walk_custom_counter)
								self.walk_custom_counter += 1
								self.last_crab_walk_custom_stamp = time.time()
								
							if (self.walk_custom_counter >= DATA_POINT_ALL):
								self.walk_custom_counter = 0

							self.allow_reset = True
							driver.RunServoInTime(acc_time=0, finish_time=self.delay_cb_time)

					self._print("delay_time {:.4f} ang_index: {:d} move_norm: {:.2f} move_ang: {:.1f} thr: {:.1f} str: {:.1f} h_cmd: {:.2f} p_cmd: {:.2f}".format(\
							self.delay_cb_time, self.ang_index, self.move_norm, self.move_ang, \
							self.thr_axis, self.str_axis, \
							self.height_cmd, self.pitch_cmd))

					self.reset_all_resetMotion_params()

				elif (abs(self.turn_axis) > 0.0):

					if (self.pitch_cmd != 0.0):
						self.prev_pitch_cmd = self.pitch_cmd
						self.pitch_cmd = 0.0
						self.setCustomHeightHomePose()

					self.delay_turning_time = self.map_with_limit(abs(self.turn_axis), 0.0, 1.0, self.biggest_delay, self.smallest_delay)

					if ((time.time() - self.last_turn_stamp) >= self.delay_turning_time/1000):

						if self.turn_counter < DATA_POINT_TURN_ALL:
							if self.turn_axis > 0.0:
								if self.height_cmd == 0.0:
									
									self.setTurningRightAngle(self.turn_counter)
								else:
									# self.setCustomTurningCWLegAngle(self.turn_counter)
									self.setCustomTurningRightLegAngle(self.turn_counter)
								self.turn_counter += 1

							else:
								if self.height_cmd == 0.0:
									self.setTurningLeftAngle(self.turn_counter)
								else:
									# self.setCustomTurningCCWLegAngle(self.turn_counter)
									self.setCustomTurningLeftLegAngle(self.turn_counter)
								self.turn_counter += 1

							self.last_turn_stamp = time.time()
							driver.RunServoInTime(acc_time=0, finish_time=self.delay_cb_time)
							self.allow_reset = True

						if self.turn_counter >= DATA_POINT_TURN_ALL:
							self.turn_counter = 0

					self._print("delay_time {:.4f} turn_counter {:d} turn_axis: {:.1f}".format(\
						self.delay_turning_time, self.turn_counter, self.turn_axis))

					self.reset_all_resetMotion_params()

			###########################
			### Orientation Control ###
			###########################
			elif self.control_mode == 1:

				if (self.pitch_cmd != 0.0):
					self.prev_pitch_cmd = self.pitch_cmd
					self.pitch_cmd = 0.0
					self.setCustomHeightHomePose()

				roll_cmd = self.map_with_limit(self.thr_axis, -1.0, 1.0, -self.max_body_rotate_deg, self.max_body_rotate_deg)
				pitch_cmd = self.map_with_limit(self.str_axis, -1.0, 1.0, -self.max_body_rotate_deg, self.max_body_rotate_deg)
				yaw_cmd = self.map_with_limit(self.turn_axis, -1.0, 1.0, self.max_yaw_rotate_deg, -self.max_yaw_rotate_deg)
				
				if self.height_cmd == 0.0:
					self.setBodyControlRotationAngle(roll_cmd, pitch_cmd, yaw_cmd)
				else:
					self.setBodyControlRotationAngle_CustomHeight(roll_cmd, pitch_cmd, yaw_cmd)

				if (time.time() - self.last_body_rotation_stamp) > self.delay_body_rotaiton/1000:
					driver.RunServoInTime(acc_time=0, finish_time=self.delay_body_rotaiton)
					self.last_body_rotation_stamp = time.time()

				self.allow_reset = True

			#################################
			### Rough Walking (Wave-Gait) ###
			#################################
			elif self.control_mode == 2:
				self.prev_ang_index = self.ang_index
				# self.delay_wg_time = self.map_with_limit(self.move_norm, 0.0, 1.0, self.wg_biggest_delay, self.wg_smallest_delay)
				self.ang_index = self.find_ang_index(self.move_ang)

				if ((time.time() - self.last_wave_gait_stamp) >= self.wg_constant_delay/1000) and ((self.move_norm > 0.1)):

					if ((self.wg_walk_counter >= WG_DATA_POINT_ALL) or (self.wg_walk_counter == 0) or (self.ang_index != self.prev_ang_index)):
						self.wg_walk_counter = 0

						XYZ_1 = lib.XYZ_WaveGait_realTime(1, self.ang_index, -self.turn_axis, self.customHeight_home_xyz)
						XYZ_2 = lib.XYZ_WaveGait_realTime(2, self.ang_index, -self.turn_axis, self.customHeight_home_xyz)
						XYZ_3 = lib.XYZ_WaveGait_realTime(3, self.ang_index, -self.turn_axis, self.customHeight_home_xyz)
						XYZ_4 = lib.XYZ_WaveGait_realTime(4, self.ang_index, self.turn_axis, self.customHeight_home_xyz)
						XYZ_5 = lib.XYZ_WaveGait_realTime(5, self.ang_index, self.turn_axis, self.customHeight_home_xyz)
						XYZ_6 = lib.XYZ_WaveGait_realTime(6, self.ang_index, self.turn_axis, self.customHeight_home_xyz)

						self.WG_THETA1_1, self.WG_THETA2_1, self.WG_THETA3_1 = lib.invKinArray_to_ThetaArray(np.transpose(XYZ_1))
						self.WG_THETA1_2, self.WG_THETA2_2, self.WG_THETA3_2 = lib.invKinArray_to_ThetaArray(np.transpose(XYZ_2))
						self.WG_THETA1_3, self.WG_THETA2_3, self.WG_THETA3_3 = lib.invKinArray_to_ThetaArray(np.transpose(XYZ_3))
						self.WG_THETA1_4, self.WG_THETA2_4, self.WG_THETA3_4 = lib.invKinArray_to_ThetaArray(np.transpose(XYZ_4))
						self.WG_THETA1_5, self.WG_THETA2_5, self.WG_THETA3_5 = lib.invKinArray_to_ThetaArray(np.transpose(XYZ_5))
						self.WG_THETA1_6, self.WG_THETA2_6, self.WG_THETA3_6 = lib.invKinArray_to_ThetaArray(np.transpose(XYZ_6))


					if (self.wg_walk_counter < WG_DATA_POINT_ALL):

						driver.joint_deg_cmd[1] = np.degrees(self.WG_THETA1_1[self.wg_walk_counter])
						driver.joint_deg_cmd[2] = np.degrees(self.WG_THETA2_1[self.wg_walk_counter])
						driver.joint_deg_cmd[3] = np.degrees(self.WG_THETA3_1[self.wg_walk_counter])

						driver.joint_deg_cmd[4] = np.degrees(self.WG_THETA1_2[self.wg_walk_counter])
						driver.joint_deg_cmd[5] = np.degrees(self.WG_THETA2_2[self.wg_walk_counter])
						driver.joint_deg_cmd[6] = np.degrees(self.WG_THETA3_2[self.wg_walk_counter])

						driver.joint_deg_cmd[7] = np.degrees(self.WG_THETA1_3[self.wg_walk_counter])
						driver.joint_deg_cmd[8] = np.degrees(self.WG_THETA2_3[self.wg_walk_counter])
						driver.joint_deg_cmd[9] = np.degrees(self.WG_THETA3_3[self.wg_walk_counter])

						driver.joint_deg_cmd[10] = np.degrees(self.WG_THETA1_4[self.wg_walk_counter])
						driver.joint_deg_cmd[11] = np.degrees(self.WG_THETA2_4[self.wg_walk_counter])
						driver.joint_deg_cmd[12] = np.degrees(self.WG_THETA3_4[self.wg_walk_counter])

						driver.joint_deg_cmd[13] = np.degrees(self.WG_THETA1_5[self.wg_walk_counter])
						driver.joint_deg_cmd[14] = np.degrees(self.WG_THETA2_5[self.wg_walk_counter])
						driver.joint_deg_cmd[15] = np.degrees(self.WG_THETA3_5[self.wg_walk_counter])

						driver.joint_deg_cmd[16] = np.degrees(self.WG_THETA1_6[self.wg_walk_counter])
						driver.joint_deg_cmd[17] = np.degrees(self.WG_THETA2_6[self.wg_walk_counter])
						driver.joint_deg_cmd[18] = np.degrees(self.WG_THETA3_6[self.wg_walk_counter])


						driver.RunServoInTime(acc_time=0, finish_time=self.wg_constant_delay)

						self.wg_walk_counter += 1
						self.last_wave_gait_stamp = time.time()
						
					self.allow_reset = True

			#######################
			### Inspection Mode ###
			#######################
			elif self.control_mode == 3:

				self.allow_reset = False

				if self.got_leg2_xyz_cmd:
					self.leg2_theta1, self.leg2_theta2, self.leg2_theta3 = lib.inv(self.leg2_xyz_cmd[0], self.leg2_xyz_cmd[1], self.leg2_xyz_cmd[2])
					self.got_leg2_xyz_cmd = False
					driver.joint_deg_cmd[4] = np.degrees(self.leg2_theta1)
					driver.joint_deg_cmd[5] = np.degrees(self.leg2_theta2)
					driver.joint_deg_cmd[6] = np.degrees(self.leg2_theta3)
					driver.RunServoInTimeByLeg(acc_time=self.inspect_movement_time//2, finish_time=self.inspect_movement_time, leg_no=2)
					self.insp_leg_xyz = {'x': self.leg2_xyz_cmd[0], 'y': self.leg2_xyz_cmd[1], 'z': self.leg2_xyz_cmd[2]}
					self.insp_leg_no = 2
					self.insp_sv_list = [4,5,6]

					
				if self.got_leg3_xyz_cmd:
					self.leg3_theta1, self.leg3_theta2, self.leg3_theta3 = lib.inv(self.leg3_xyz_cmd[0], self.leg3_xyz_cmd[1], self.leg3_xyz_cmd[2])
					self.got_leg3_xyz_cmd = False
					driver.joint_deg_cmd[7] = np.degrees(self.leg3_theta1)
					driver.joint_deg_cmd[8] = np.degrees(self.leg3_theta2)
					driver.joint_deg_cmd[9] = np.degrees(self.leg3_theta3)
					driver.RunServoInTimeByLeg(acc_time=self.inspect_movement_time//2, finish_time=self.inspect_movement_time, leg_no=3)
					self.insp_leg_xyz = {'x': self.leg3_xyz_cmd[0], 'y': self.leg3_xyz_cmd[1], 'z': self.leg3_xyz_cmd[2]}
					self.insp_leg_no = 3
					self.insp_sv_list = [7,8,9]

				## when receive height_cmd to adjust height
				if self.got_height_cmd_inspection:
					self.got_height_cmd_inspection = False
					self._print("height_cmd_inspection {}".format(self.height_cmd))

					driver.ReadPosition()

					leg1_x, leg1_y, leg1_z = lib.fwd(np.radians(driver.joint_position[1]) , np.radians(driver.joint_position[2]) , np.radians(driver.joint_position[3]))
					leg4_x, leg4_y, leg4_z = lib.fwd(np.radians(driver.joint_position[10]) , np.radians(driver.joint_position[11]) , np.radians(driver.joint_position[12]))
					leg5_x, leg5_y, leg5_z = lib.fwd(np.radians(driver.joint_position[13]) , np.radians(driver.joint_position[14]) , np.radians(driver.joint_position[15]))
					leg6_x, leg6_y, leg6_z = lib.fwd(np.radians(driver.joint_position[16]) , np.radians(driver.joint_position[17]) , np.radians(driver.joint_position[18]))

					new_leg1_z = self.INSPECT_Z_Home - self.height_cmd
					new_leg4_z = self.INSPECT_Z_Home - self.height_cmd
					new_leg5_z = self.INSPECT_Z_Home - self.height_cmd
					new_leg6_z = self.INSPECT_Z_Home - self.height_cmd

					leg1_theta1, leg1_theta2, leg1_theta3 = lib.inv(leg1_x, leg1_y, new_leg1_z)
					leg4_theta1, leg4_theta2, leg4_theta3 = lib.inv(leg4_x, leg4_y, new_leg4_z)
					leg5_theta1, leg5_theta2, leg5_theta3 = lib.inv(leg5_x, leg5_y, new_leg5_z)
					leg6_theta1, leg6_theta2, leg6_theta3 = lib.inv(leg6_x, leg6_y, new_leg6_z)

					driver.joint_deg_cmd[1] = np.degrees(leg1_theta1)
					driver.joint_deg_cmd[2] = np.degrees(leg1_theta2)
					driver.joint_deg_cmd[3] = np.degrees(leg1_theta3)

					driver.joint_deg_cmd[10] = np.degrees(leg4_theta1)
					driver.joint_deg_cmd[11] = np.degrees(leg4_theta2)
					driver.joint_deg_cmd[12] = np.degrees(leg4_theta3)

					driver.joint_deg_cmd[13] = np.degrees(leg5_theta1)
					driver.joint_deg_cmd[14] = np.degrees(leg5_theta2)
					driver.joint_deg_cmd[15] = np.degrees(leg5_theta3)

					driver.joint_deg_cmd[16] = np.degrees(leg6_theta1)
					driver.joint_deg_cmd[17] = np.degrees(leg6_theta2)
					driver.joint_deg_cmd[18] = np.degrees(leg6_theta3)

					driver.RunServoInTimeByLeg(acc_time=self.inspect_movement_time//2, finish_time=self.inspect_movement_time, leg_no=1)
					driver.RunServoInTimeByLeg(acc_time=self.inspect_movement_time//2, finish_time=self.inspect_movement_time, leg_no=4)
					driver.RunServoInTimeByLeg(acc_time=self.inspect_movement_time//2, finish_time=self.inspect_movement_time, leg_no=5)
					driver.RunServoInTimeByLeg(acc_time=self.inspect_movement_time//2, finish_time=self.inspect_movement_time, leg_no=6)

				if (time.time() - self.last_read_fb_stamp) > 0.1:
					
					self.readCurrentAndPosition()

					leg2_xyz_fb_msg = Float32MultiArray()
					leg3_xyz_fb_msg = Float32MultiArray()
					leg2_xyz_fb_msg.data = [self.leg2_x_fb, self.leg2_y_fb, self.leg2_z_fb]
					leg3_xyz_fb_msg.data = [self.leg3_x_fb, self.leg3_y_fb, self.leg3_z_fb]

					self.leg2_xyz_fb_pub.publish(leg2_xyz_fb_msg)
					self.leg3_xyz_fb_pub.publish(leg3_xyz_fb_msg)

					self.last_read_fb_stamp = time.time()

				if self.got_inspect_start:

					## status: -1->cancel or failed, 0->touching, 1->inspecting, 2->return result

					######################
					### Touching check ###
					######################
					if self.insp_step == 0:

						self.pub_insp_status(0)

						x_touch = self.insp_leg_xyz['x']
						y_touch = self.insp_leg_xyz['y']
						z_touch = self.insp_leg_xyz['z'] - self.z_touch_counter

						theta1, theta2, theta3 = lib.inv(x_touch, y_touch, z_touch)

						driver.joint_deg_cmd[self.insp_sv_list[0]] = np.degrees(theta1)
						driver.joint_deg_cmd[self.insp_sv_list[1]] = np.degrees(theta2)
						driver.joint_deg_cmd[self.insp_sv_list[2]] = np.degrees(theta3)
						driver.RunServoInTimeByLeg(acc_time=self.inspect_touch_time//2, finish_time=self.inspect_touch_time, leg_no=self.insp_leg_no)

						time.sleep(self.inspect_touch_time/1000)
						self.readCurrentAndPosition()

						self._print("servo{:d}: cur: {:.1f} pos: {:.1f} | servo{:d} cur: {:.1f} pos: {:.1f}".format(\
							self.insp_sv_list[1], driver.joint_cur_pos[self.insp_sv_list[1]]['cur'], driver.joint_cur_pos[self.insp_sv_list[1]]['pos'],\
							self.insp_sv_list[2], driver.joint_cur_pos[self.insp_sv_list[2]]['cur'], driver.joint_cur_pos[self.insp_sv_list[2]]['pos']))

						if self.servo_cur_fb[self.insp_sv_list[1]] < -30.0:
							self._print("touched!")
							self._print("{} {} {}".format(x_touch, y_touch, z_touch))
							x_touched = x_touch
							y_touched = y_touch
							z_touched = z_touch + 4 ## lift up a bit to release stress
							theta1, theta2, theta3 = lib.inv(x_touched, y_touched, z_touched)
							driver.joint_deg_cmd[self.insp_sv_list[0]] = np.degrees(theta1)
							driver.joint_deg_cmd[self.insp_sv_list[1]] = np.degrees(theta2)
							driver.joint_deg_cmd[self.insp_sv_list[2]] = np.degrees(theta3)
							driver.RunServoInTimeByLeg(acc_time=self.inspect_touch_time//2, finish_time=self.inspect_touch_time, leg_no=self.insp_leg_no)
							time.sleep(1)
							self.insp_start_xyz = {'x': x_touched, 'y': y_touched, 'z': z_touched}
							self.insp_target_xyz = {'x': x_touched, 'y': y_touched, 'z': z_touched-self.z_inspect_limit}

							self.insp_step = 1
							self.z_touch_counter = 0

						if self.z_touch_counter <= self.z_touch_limit:
							self.z_touch_counter += 1
						else:
							self.insp_step = -1

					###########################
					### Inspection starting ###
					###########################
					elif self.insp_step == 1:

						self.pub_insp_status(1)

						x_insp = self.insp_start_xyz['x']
						y_insp = self.insp_start_xyz['y']
						z_insp = self.insp_start_xyz['z'] - self.z_inspect_counter

						# self._print(x_insp, y_insp, z_insp)

						theta1, theta2, theta3 = lib.inv(x_insp, y_insp, z_insp)

						driver.joint_deg_cmd[self.insp_sv_list[0]] = np.degrees(theta1)
						driver.joint_deg_cmd[self.insp_sv_list[1]] = np.degrees(theta2)
						driver.joint_deg_cmd[self.insp_sv_list[2]] = np.degrees(theta3)
						driver.RunServoInTimeByLeg(acc_time=self.inspect_touch_time//2, finish_time=self.inspect_touch_time, leg_no=self.insp_leg_no)

						time.sleep(self.inspect_touch_time/1000)
						self.readCurrentAndPosition()

						if self.z_inspect_counter <= self.z_inspect_limit:
							self.z_inspect_counter += 1
						else:
							self.insp_step = 2
							self.z_inspect_counter = 0

					#######################
					### Estimate Result ###
					#######################
					elif self.insp_step == 2:
						last_the1 = np.radians(driver.joint_cur_pos[self.insp_sv_list[0]]['pos'])
						last_the2 = np.radians(driver.joint_cur_pos[self.insp_sv_list[1]]['pos'])
						last_the3 = np.radians(driver.joint_cur_pos[self.insp_sv_list[2]]['pos'])
						last_x, last_y, last_z = lib.fwd(last_the1, last_the2, last_the3)
						last_cur2 = driver.joint_cur_pos[self.insp_sv_list[1]]['cur']
						last_cur3 = driver.joint_cur_pos[self.insp_sv_list[2]]['cur']

						diff_x = abs((last_x - self.insp_target_xyz['x']))
						diff_y = abs((last_y - self.insp_target_xyz['y']))
						diff_z = abs((last_z - self.insp_target_xyz['z']))

						self._print("target_xyz {}".format(self.insp_target_xyz))
						self._print("last_xyz {} {} {}".format(last_x, last_y, last_z))
						self._print("last_cur {} {}".format(last_cur2, last_cur3))
						self._print("diff {} {} {}".format(diff_x, diff_y, diff_z))

						train_data_msg = Float32MultiArray()
						train_data_msg.data = [diff_x, diff_y, diff_z, last_cur2, last_cur3]
						self.train_data_pub.publish(train_data_msg)

						self.got_inspect_start = False
						self.insp_step = 0

						## return back the leg
						theta1, theta2, theta3 = lib.inv(self.insp_start_xyz['x'], self.insp_start_xyz['y'], self.insp_start_xyz['z']+10)
						driver.joint_deg_cmd[self.insp_sv_list[0]] = np.degrees(theta1)
						driver.joint_deg_cmd[self.insp_sv_list[1]] = np.degrees(theta2)
						driver.joint_deg_cmd[self.insp_sv_list[2]] = np.degrees(theta3)
						driver.RunServoInTimeByLeg(acc_time=self.inspect_movement_time//2, finish_time=self.inspect_movement_time, leg_no=self.insp_leg_no)

						self.insp_leg_xyz['x'] = self.insp_start_xyz['x']
						self.insp_leg_xyz['y'] = self.insp_start_xyz['y']
						self.insp_leg_xyz['z'] = self.insp_start_xyz['z']+10

						pred = svm_model.predict(np.array([[diff_x, diff_y, diff_z, last_cur2, last_cur3]])).item()
						self._print("prediction {}".format(pred))

						if pred == 0:
							self.pub_insp_status(2)
						elif pred == 1:
							self.pub_insp_status(3)

						

					#######################
					### Touching failed ###
					#######################
					elif self.insp_step == -1:
						self.got_inspect_start = False
						self.insp_step = 0
						self._print("failed, cannot touch object")

						self.pub_insp_status(-1)





		###########################################
		### Reset Motion                        ###
		### release servo tension in some pose  ###
		### when stop moving robot suddenly     ###
		###########################################
		cond_to_reset = ((time.time() - self.last_crab_walk_stamp) > self.reset_timeout) and \
						((time.time() - self.last_turn_stamp) > self.reset_timeout) and \
						((time.time() - self.last_body_rotation_stamp) > self.reset_timeout) and \
						((time.time() - self.last_crab_walk_custom_stamp) > self.reset_timeout) and \
						((time.time() - self.last_wave_gait_stamp) > self.reset_timeout) and \
						(self.allow_reset)

		if cond_to_reset:

			self.robot_mode = "RSET"
			self.resetMotion_toHome()
			self.pub_insp_status(-2)
			
					
		###################
		### Logging out ###
		###################
		# if self.robot_mode == "CRAB":
		# 	counter_log = self.walk_counter
		# 	delay_log = self.delay_cb_time
		# elif self.robot_mode == "TURN":
		# 	counter_log = self.turn_counter
		# 	delay_log = self.delay_turning_time
		# else:
		# 	counter_log = 0
		# 	delay_log = 0

		# self._print("mode: {} ch1: {:d} ch2: {:d} ch4: {:d} counter: {:d} delay: {:d} ang_index: {:d} allow_rst: {} leg_no: {:d} rst_moving: {} home_moving: {} r_cmd: {:.2f} p_cmd: {:.2f} y_cmd: {:.2f}".format(\
		# 	self.robot_mode, self.sbus_str, self.sbus_thr, self.sbus_yaw,\
		# 	counter_log, delay_log, self.ang_index, \
		# 	self.allow_reset, self.leg_no, self.reset_moving, self.home_moving, \
		# 	self.roll_cmd_deg, self.pitch_cmd_deg, self.yaw_cmd_deg))





			


def main(args=None):
	rclpy.init(args=None)
	node = SpiderBotControl()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()