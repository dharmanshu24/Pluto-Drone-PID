#!/usr/bin/env python

# The required packages are imported here
from plutodrone.msg import *
from pid_tune.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import *
from plutodrone.srv import *  # for yaw
import rospy
import time


class DroneFly():
	"""docstring for DroneFly"""
	def __init__(self):
		rospy.init_node('pluto_fly', disable_signals = True)
		self.pluto_cmd = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)
		rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)
		# To tune the drone during runtime
		rospy.Subscriber('/pid_tuning_altitude', PidTune, self.set_pid_alt)
		rospy.Subscriber('/pid_tuning_roll', PidTune, self.set_pid_roll)
		rospy.Subscriber('/pid_tuning_pitch', PidTune, self.set_pid_pitch)
		rospy.Subscriber('/pid_tuning_yaw', PidTune, self.set_pid_yaw)
		# rospy.Subscriber('/yaw', Float32, self.set_drone_yaw)

		self.cmd = PlutoMsg()

		# Position to hold.
		self.wp_x = -6
		self.wp_y = 0
		self.wp_z = 14.0
		self.wp_yaw = 25

		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1000
		self.cmd.plutoIndex = 0

		self.drone_x = 0.0
		self.drone_y = 0.0
		self.drone_z = 0.0
		self.drone_yaw = 0.0

		# PID constants for Roll
		self.kp_roll = 5.2
		self.ki_roll = 1.7
		self.kd_roll = 7.5

		# PID constants for Pitch
		self.kp_pitch = 5.0
		self.ki_pitch = 1.00
		self.kd_pitch = 7.3

		# PID constants for Yaw
		self.kp_yaw = 10
		self.ki_yaw = 0.0
		self.kd_yaw = 0.0

		# PID constants for Throttle
		self.kp_throt = 20
		self.ki_throt = 2
		self.kd_throt = 0.5

		# Correction values after PID is computed
		self.correct_roll = 0.0
		self.correct_pitch = 0
		self.correct_yaw = 0.0
		self.correct_throt = 0

		# Loop time for PID computation. You are free to experiment with this
		self.last_time = 0
		self.loop_time = 0.032
		self.last_error_throt = 0
		self.last_error_pitch = 0
		self.last_error_roll = 0
		self.last_error_yaw = 0
		self.iError_pitch = 0
		self.iError_roll = 0
		self.iError_throt = 0
		self.iError_yaw = 0
		self.windup_guard = 50
		self.last_time_throt = time.time()

		rospy.sleep(.1)


	def arm(self):
		self.cmd.rcAUX4 = 1500
		self.cmd.rcThrottle = 1000
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)

	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)

	def position_hold(self):

		rospy.sleep(2)

		print "disarm"
		self.disarm()
		rospy.sleep(.2)
		print "arm"
		self.arm()
		rospy.sleep(.1)
		self.norm_pid()

		while True:
			# Waiting till loop time is achieved
			while True:
				if (time.time() - self.last_time) >= self.loop_time:
					self.last_time = time.time()
					break
			# try:
			# 	time.sleep(time.time() - self.last_time - self.loop_time);
			# except Exception as e:
			# 	pass
			self.last_time = time.time()
			self.calc_pid()
			# Check your X and Y axis. You MAY have to change the + and the -.
			# We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
			pitch_value = int(1540 + self.correct_pitch) # x
			self.cmd.rcPitch = self.limit(pitch_value, 1600, 1400)

			roll_value = int(1550 + self.correct_roll) # y
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)

			throt_value = int(1540 - self.correct_throt)
			self.cmd.rcThrottle = self.limit(throt_value, 1850, 1350)

			# throt_value = int(1700 - self.correct_throt)
			# self.cmd.rcThrottle = self.limit(throt_value, 1950, 1450)

			self.publish_plot_data()
			self.pluto_cmd.publish(self.cmd)


	def calc_pid(self):
		self.correct_roll, self.iError_roll, self.last_error_roll = self.pid_calc(self.wp_y, self.drone_y, self.kp_roll, self.ki_roll, self.kd_roll, self.iError_roll, self.last_error_roll, self.windup_guard, -1*self.windup_guard)
		# self.correct_yaw, self.iError_yaw, self.last_error_yaw = self.pid_calc(self.wp_yaw, self.drone_yaw, self.kp_yaw, self.ki_yaw, self.kd_yaw, self.iError_yaw, self.last_error_yaw,self.windup_guard, -1*self.windup_guard)
		self.correct_pitch, self.iError_pitch, self.last_error_pitch = self.pid_calc(self.wp_x, self.drone_x, self.kp_pitch, self.ki_pitch, self.kd_pitch, self.iError_pitch, self.last_error_pitch, self.windup_guard, -1*self.windup_guard)
		self.correct_throt, self.iError_throt, self.last_error_throt = self.pid_calc(self.wp_z, self.drone_z, self.kp_throt, self.ki_throt, self.kd_throt, self.iError_throt, self.last_error_throt, self.windup_guard, -1*self.windup_guard)
		# self.correct_yaw, self.iError_yaw, self.last_error_yaw = self.pid_calc(self.wp_yaw, self.drone_yaw, self.kp_yaw, self.ki_yaw, self.kd_yaw, self.iError_yaw, self.last_error_yaw,self.windup_guard, -1_self.windup_guard)



	def pid_calc(self, fPos, curPos, kp, ki, kd, iError, lastPos, windup_max, windup_min):
		pError = fPos - curPos
		iError += pError

		iError = self.limit(iError, windup_max, windup_min)
		dError = curPos - lastPos
		pid = kp*pError + ki*iError - kd*dError
		# print('pid = ',pid, 'kp = ', kp*pError, "ki = ", ki*iError, "kd = ", kd*dError)
		return pid, iError, curPos


	# def pid_roll(self):
	#
	# 	#Compute Roll PID here
	# 	pError = self.wp_y - self.drone_y   # Propotional error
	# 	self.iError_roll += pError  # integral error
	#
	# 	self.iError_roll = self.limit(self.iError_roll, self.windup_guard, -1*self.windup_guard) # keeps integral error between windup_guard
	#
	# 	dError = (self.drone_y - self.last_error_roll)   # diffrential error
	# 	self.correct_roll = self.kp_roll*pError + self.ki_roll*self.iError_roll - self.kd_roll*dError
	# 	self.last_error_roll = self.drone_y
	#
	#
	# def pid_pitch(self):
	#
	# 	#Compute Pitch PID here
	# 	pError = self.wp_x - self.drone_x   # Propotional error
	# 	self.iError_pitch += pError  # integral error
	#
	# 	self.iError_pitch = self.limit(self.iError_pitch, self.windup_guard, -1*self.windup_guard) # Keeps integral error between windup guard
	#
	# 	dError = (self.drone_x - self.last_error_pitch)  # diffrential error
	# 	self.correct_pitch = self.kp_pitch*pError + self.ki_pitch*self.iError_pitch - self.kd_pitch*dError
	# 	self.last_error_pitch = self.drone_x
	#
	# def pid_throt(self):
	# 	#Compute Throttle PID here
	# 	pError = self.wp_z - self.drone_z   # Propotional error
	# 	self.iError_throt += pError   # integral error
	#
	# 	self.iError_roll = self.limit(self.iError_throt, self.windup_guard, -1*self.windup_guard) # Keeps Integral error between |windup_guard|
	#
	# 	dError = (self.drone_z - self.last_error_throt)  # diffrential error
	# 	self.correct_throt = self.kp_throt*pError + self.ki_throt*self.iError_throt - self.kd_throt*dError
	# 	self.last_error_throt = self.drone_z

	def limit(self, input_value, max_value, min_value):

		#Use this function to limit the maximum and minimum values you send to your drone

		if input_value > max_value:
			return max_value
		if input_value < min_value:
			return min_value
		else:
			return input_value

	#You can use this function to publish different information for your plots
	def publish_plot_data(self):
		self.error_x = rospy.Publisher('/error_x', Float32, queue_size=10)
		self.error_x.publish(self.wp_x-self.drone_x)
		self.error_y = rospy.Publisher('/error_y', Float32, queue_size=10)
		self.error_y.publish(self.wp_y-self.drone_y)
		self.error_z = rospy.Publisher('/error_z', Float32, queue_size=10)
		self.error_z.publish(self.wp_z-self.drone_z)
		self.error_yaw = rospy.Publisher('/error_yaw', Float32, queue_size=10)
		self.error_yaw.publish(self.wp_yaw-self.drone_yaw)
		thrt = rospy.Publisher('/throt_pid', Float32, queue_size=10)
		thrt.publish(self.correct_throt)
		roll = rospy.Publisher('/roll_pid', Float32, queue_size=10)
		roll.publish(self.correct_roll)
		# print self.correct_roll
		pitch = rospy.Publisher('/pitch_pid', Float32, queue_size=10)
		pitch.publish(self.correct_pitch)
		yaw = rospy.Publisher('/yaw_pid', Float32, queue_size=10)
		yaw.publish(self.correct_yaw)



	def set_pid_alt(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Altitude

		self.kp_throt = pid_val.Kp/100
		self.ki_throt = pid_val.Ki/100
		self.kd_throt = pid_val.Kd/100

	def set_pid_roll(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Roll

		self.kp_roll = pid_val.Kp
		self.ki_roll = pid_val.Ki
		self.kd_roll = pid_val.Kd

	def set_pid_pitch(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Pitch

		self.kp_pitch = pid_val.Kp
		self.ki_pitch = pid_val.Ki
		self.kd_pitch = pid_val.Kd

	def set_pid_yaw(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Yaw

		self.kp_yaw = pid_val.Kp
		self.ki_yaw = pid_val.Ki
		self.kd_yaw = pid_val.Kd


	def set_drone_yaw(self, drone_y):
		self.drone_yaw = float(str(drone_y).split(" ")[1])

	def get_pose(self,pose):

		#This is the subscriber function to get the whycon poses
		#The x, y and z values are stored within the drone_x, drone_y and the drone_z variables
		# self.drone_yaw =
		self.drone_x = pose.poses[0].position.x
		self.drone_y = pose.poses[0].position.y
		self.drone_z = pose.poses[0].position.z

		# print(pose.poses)
		# data = rospy.Service('PlutoService', PlutoPilot, self.access_data)

	def norm_pid(self):
		# Normalises K values, no need for loop time in calculating pid_val
		self.ki_yaw *= self.loop_time
		self.kd_yaw /= self.loop_time
		self.ki_roll *= self.loop_time
		self.kd_roll /= self.loop_time
		self.ki_pitch *= self.loop_time
		self.kd_pitch /= self.loop_time
		self.ki_throt *= self.loop_time
		self.kd_throt /= self.loop_time


if __name__ == '__main__':
	while not rospy.is_shutdown():
		try:
			temp = DroneFly()
			temp.position_hold()
			rospy.spin()
		except KeyboardInterrupt:
			temp.disarm()
			break
