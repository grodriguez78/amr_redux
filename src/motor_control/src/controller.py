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
		self.encPins = [13, 6, 24, 23] 	# GPIO pins used to read encoders
		self.mcPins = [16, 20, 19, 26] 	# GPIO pins used for motor control

		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.mcPins, GPIO.OUT, initial=GPIO.LOW)
		GPIO.setup(self.encPins, GPIO.IN)
	
		## Hardware specs
		
		# Encoder Pins
		self.left_clk = 6
		self.left_dt = 13
		self.right_clk = 23
		self.right_dt = 24

		# Control pins
		self.left_fwd = GPIO.PWM(26, 100)
		self.left_bkwds = GPIO.PWM(19, 100)
		self.right_fwd = GPIO.PWM(16, 100)
		self.right_bkwds = GPIO.PWM(20, 100)

		# Angular distance of a single encoder step, in radians
		self.enc_step_size = 2 * np.pi / 30

		# Calculate bouncetime (ms) from max angular velocity
		self.bouncetime = int(1000 * 0.5 * self.enc_step_size / 20)

		# Storage for past encoder trigger times
		self.last_trig_left = 0.0
		self.last_trig_right = 0.0

		# Wheel velocities
		self.w_left_meas = 0.0
		self.w_right_meas = 0.0


	def encoder_direction(self, wheel):
		""" Determine encoder direction from the current state of clk and dt pins
			
			clk 	pin number for an encoder's clk output
			dt 		pin number for an encoder's dt output

			return   1 - clockwise direction
				    -1 - counter-clockwise direction
		"""

		assert wheel in ['left', 'right'], "Wheel can only be left or right!"

		# Wheel is rotating clockwise
		if GPIO.input(getattr(self, wheel + '_clk')) != GPIO.input(getattr(self, wheel + '_dt')):
			direction = 1

		# Wheel is rotating counter-clockwise
		else:
			direction = -1

		return direction

	def left_up(self, channel):
		""" Note the time the left encoder's clk went high and determine which
			direction the left wheel is turning
		"""

		# Record trigger time
		t_trig = time.time()

		# Determine rotation direction
		direction = self.encoder_direction('left')

		# Calculate angular velocity
		d_t = t_trig - self.last_trig_left
		self.w_left = direction * KY_040_STEP / d_t

		# Reset recorded trigger time
		self.last_trig_left = t_trig

		print "Left Wheel Angular Velocity: %0.2f"%self.w_left


	def right_up(self, channel):
		""" Note the time the right encoder's clk went high and determine which
			direction the wheel is turning
		"""

		# Record trigger time
		t_trig = time.time()

		# Determine rotation direction
		direction = self.encoder_direction('right')

		# Calculate measured angular velocity
		d_t = t_trig - self.last_trig_right
		self.w_right_meas = direction * KY_040_STEP / d_t

		# Reset recorded trigger time
		self.last_trig_right = t_trig

		print "Right Wheel Angular Velocity: %0.2f"%self.w_right_meas


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

		s = rospy.Subscriber("robot_dynamics", WheelVelocity, callback)

		rate = rospy.Rate(10) # 10hz

		self.last_trig_left = time.time()
		self.last_trig_right = time.time()

		# Setup encoder interrupt callbacks
		GPIO.add_event_detect(self.left_clk, GPIO.RISING, callback= self.left_up, bouncetime= BOUNCETIME)
		GPIO.add_event_detect(self.right_clk, GPIO.RISING, callback= self.right_up, bouncetime= BOUNCETIME)

		while not rospy.is_shutdown():
			rate.sleep()

if __name__ == '__main__':
	try:
		controller = Controller()
		controller.run()
	except rospy.ROSInterruptException:
		GPIO.cleanup()
		pass
