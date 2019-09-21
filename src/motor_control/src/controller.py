#!/usr/bin/env python

import rospy
import time
import numpy as np

import RPi.GPIO as GPIO

from motor_control.msg import WheelVelocity
from utils import scale_velocity


class Controller():

	def __init__(self):
		""" Initialize class

		"""

		# Setup GPIO
		self.mcPins = [16, 20, 19, 26] 	# GPIO pins used for motor control

		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.mcPins, GPIO.OUT, initial=GPIO.LOW)
	
		## Hardware specs

		# Control pins
		self.left_fwd = GPIO.PWM(26, 100)
		self.left_bkwds = GPIO.PWM(19, 100)
		self.right_fwd = GPIO.PWM(16, 100)
		self.right_bkwds = GPIO.PWM(20, 100)


	def set_wheel_velocities(self, w_left, w_right):
		""" Drive left and right wheels at angular velocities w_l, w_r

		"""

		## Left wheel
		if w_left > 0:
			left_bkwds.stop()
			left_fwd.start(scale_velocity(w_left))

		elif w_left < 0:
			left_fwd.stop()
			left_bkwds.start(scale_velocity(w_left))

		else: 
			left_fwd.stop()
			left_bkwds.stop()

		## Right wheel
		if w_right > 0:
			right_bkwds.stop()
			right_fwd.start(scale_velocity(w_right))

		elif w_right < 0:
			right_fwd.stop()
			right_bkwds.start(scale_velocity(w_right))

		else:
			right_fwd.stop()
			right_bkwds.stop()


	def callback(self, data):
		rospy.loginfo(rospy.get_caller_id() + " w_left: %d", data.w_left)
		rospy.loginfo(rospy.get_caller_id() + " w_right: %d", data.w_right)

		self.set_wheel_velocities(data.w_left, data.w_right)


	def run(self):
		## Convert angular velocity signal into individual PWM signals
		rospy.init_node('controller', anonymous=True)

		s = rospy.Subscriber("robot_dynamics", WheelVelocity, self.callback)

		rate = rospy.Rate(10) # 10hz

		while not rospy.is_shutdown():
			rate.sleep()

if __name__ == '__main__':
	try:
		controller = Controller()
		controller.run()
	except rospy.ROSInterruptException:
		GPIO.cleanup()
		pass
