from SpiderBotDriver import *
from SpiderBotLib import *
import time

import os

if os.name == 'nt':
	import msvcrt
	def getch():
		return msvcrt.getch().decode()
else:
	import sys, tty, termios
	fd = sys.stdin.fileno()
	old_settings = termios.tcgetattr(fd)
	def getch():
		try:
			tty.setraw(sys.stdin.fileno())
			ch = sys.stdin.read(1)
		finally:
			termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
		return ch

h = SpiderBotLib()
h.generate_crabWalkingLUT()
servo1_home = np.degrees(h.theta1_home)
servo2_home = np.degrees(h.theta2_home)
servo3_home = np.degrees(h.theta3_home)

print(servo1_home, servo2_home, servo3_home)

a = SpiderBotDriver()
a.TorqueOn()
# a.TorqueOff()


# # a.joint_deg_cmd[1] = 0.0
# # a.joint_deg_cmd[2] = 0.0
# # a.joint_deg_cmd[3] = 0.0

# a.joint_deg_cmd[1] = 0.0
# a.joint_deg_cmd[2] = -30.0
# a.joint_deg_cmd[3] = -60.0

a.joint_deg_cmd[1] = servo1_home
a.joint_deg_cmd[2] = servo2_home
a.joint_deg_cmd[3] = servo3_home

# a.RunServo()
a.RunServoInTime(50, 200)

leg1 = [0,1,2]
leg2 = [3,4,5]
leg3 = [6,7,8]
leg4 = [9,10,11]
leg5 = [12,13,14]
leg6 = [15,16,17]


print("servo1", np.degrees(h.crab_walking_LUT_THETA[0][leg1[0]]))
print("servo2", np.degrees(h.crab_walking_LUT_THETA[0][leg1[1]]))
print("servo3", np.degrees(h.crab_walking_LUT_THETA[0][leg1[2]]))

time.sleep(1)
### Go to starting point slowly
a.joint_deg_cmd[1] = np.degrees(h.crab_walking_LUT_THETA[0][leg1[0]][0])
a.joint_deg_cmd[2] = np.degrees(h.crab_walking_LUT_THETA[0][leg1[1]][0])
a.joint_deg_cmd[3] = np.degrees(h.crab_walking_LUT_THETA[0][leg1[2]][0])
a.RunServoInTime(500, 1000)

time.sleep(2)

### drag + lifting ###
finish_time_ms = 80   ## min 20  max 80
acc_time_ms = 0 #finish_time_ms//1.5
period_array = []


angle_thresh = 1.0


while True:
	period_array = []
	for i in range(h.DATA_POINT_ALL):

		start_time = time.time()
		a.joint_deg_cmd[1] = np.degrees(h.crab_walking_LUT_THETA[0][leg1[0]][i])
		a.joint_deg_cmd[2] = np.degrees(h.crab_walking_LUT_THETA[0][leg1[1]][i])
		a.joint_deg_cmd[3] = np.degrees(h.crab_walking_LUT_THETA[0][leg1[2]][i])
		a.RunServoInTime(acc_time_ms, finish_time_ms)
		# a.RunServo()
		
		# start_time = time.time()
		# while True:
		# 	a.ReadPosition()
		# 	print("servo1 cmd: {:.2f} pos: {:.2f} moving: {:d}".format(a.joint_deg_cmd[1], a.joint_position[1], a.joint_moving[1]))
		# 	print("servo2 cmd: {:.2f} pos: {:.2f} moving: {:d}".format(a.joint_deg_cmd[2], a.joint_position[2], a.joint_moving[2]))
		# 	print("servo3 cmd: {:.2f} pos: {:.2f} moving: {:d}".format(a.joint_deg_cmd[3], a.joint_position[3], a.joint_moving[3]))
		# 	print(" ")
		# 	if (abs(a.joint_deg_cmd[1] - a.joint_position[1]) < angle_thresh) and (abs(a.joint_deg_cmd[2] - a.joint_position[2]) < angle_thresh) and (abs(a.joint_deg_cmd[3] - a.joint_position[3]) < angle_thresh):
		# 		break

		# period = time.time() - start_time
		# period_array.append(period)

		time.sleep((finish_time_ms/1000)/1.0)

	# print(period_array)


### Simple 3dots ###
# p1 = [h.X_home, -100.0, h.Z_home]
# p2 = [h.X_home, 0.0, h.Z_home-(-50.0)]
# p3 = [h.X_home, 100.0, h.Z_home]
# p4 = p1

# THETA_1 = h.inv(p1[0], p1[1], p1[2])
# THETA_2 = h.inv(p2[0], p2[1], p2[2])
# THETA_3 = h.inv(p3[0], p3[1], p3[2])
# THETA_4 = h.inv(p4[0], p4[1], p4[2])

# DEG_1 = np.degrees(THETA_1)
# DEG_2 = np.degrees(THETA_2)
# DEG_3 = np.degrees(THETA_3)
# DEG_4 = np.degrees(THETA_4)

# deg1_list = [DEG_1[0], DEG_2[0], DEG_3[0], DEG_4[0]]
# deg2_list = [DEG_1[1], DEG_2[1], DEG_3[1], DEG_4[1]]
# deg3_list = [DEG_1[2], DEG_2[2], DEG_3[2], DEG_4[2]]


# print("DEG_1", DEG_1)
# print("DEG_2", DEG_2)
# print("DEG_3", DEG_3)
# print("DEG_4", DEG_4)

# finish_time_ms = 300
# acc_time_ms = finish_time_ms//3
# for i in range(len(deg1_list)):
# 	a.joint_deg_cmd[1] = deg1_list[i]
# 	a.joint_deg_cmd[2] = deg2_list[i]
# 	a.joint_deg_cmd[3] = deg3_list[i]
# 	a.RunServoInTime(acc_time_ms, finish_time_ms)

# 	time.sleep(finish_time_ms/1000)