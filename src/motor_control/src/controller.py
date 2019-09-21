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
		self.left_wheel_clk = 6
		self.left_wheel_dt = 13
		self.right_wheel_clk = 23
		self.right_wheel_dt = 24

		# Control pins
		self.left_fwd = GPIO.PWM(26, 100)
		self.left_bkwds = GPIO.PWM(19, 100)
		self.right_fwd = GPIO.PWM(16, 100)
		self.right_bkwds = GPIO.PWM(20, 100)

		# Angular distance of a single encoder step, in radians
		self.enc_step_size = 2 * np.pi / 30

		# Calculate bouncetime (ms) from max angular velocity
		self.bouncetime = int(1000 * 0.5 * self.enc_step_size / 20)

		# Encoder identifiers
		self.encoder_ids = ['left_wheel', 'right_wheel']

		# Storage for encoder step counts
		self.left_wheel_step_count = 0
		self.right_wheel_step_count = 0

		# Storage for current encoder direction
		#  1 - Clockwise
		#  0 - Unset / stopped
		# -1 - Counter-Clockwise
		self.left_wheel_dir = 0
		self.right_wheel_dir = 0

		# Accepted string values for encoder directions
		self.enc_direction_map = {
			-1: ['ccw', 'counter-clockwise']
			0:  ['stopped'],
			1:  ['cw', 'clockwise']
		}

		# Storage for encoder resets 
		self.left_wheel_reset_time = None
		self.right_wheel_reset_time = None

		# Wheel velocities
		self.left_wheel_meas_w = 0.0
		self.right_wheel_meas_w = 0.0



	def meas_encoder_direction(self, encoder_id):
		""" Measure encoder direction from the current state of dt pin.

			Due to the rising edge triggers set on encoder clk outputs, it can
			be assumed that clk is high when this function is called.
			
			clk 	pin number for an encoder's clk output
			dt 		pin number for an encoder's dt output

			return   1 - clockwise 
				    -1 - counter-clockwise 
		"""

		# Ensure specified encoder exists
		assert encoder_id in self.encoder_ids, "Specified encoder %s does not exist!"%encoder_id

		# d_t is HIGH (Encoder is rotating counter-clockwise)
		if GPIO.input(getattr(self, encoder_id + '_dt')):
			return -1

		# d_t is LOW (Encoder is rotating clockwise)
		else:
			return 1


	def record_encoder_tick(self, encoder_id):
		""" Update encoder step counter or reset if direction has changed

		"""

		# Ensure specified encoder exists
		assert encoder_id in self.encoder_ids, "Specified encoder %s does not exist!"%encoder_id

		meas_dir = self.meas_encoder_direction(encoder_id)

		# Check if encoder has changed direction
		if getattr(self, encoder_id + '_dir') != meas_dir:
			self.reset_encoder(encoder_id)
			self.set_encoder_direction(encoder_id, meas_dir)

		else:
			# Increment encoder step count
			getattr(self, encoder_id + '_step_count')

	def reset_encoder(self, encoder_id):
		""" Set encoder step count to 0 and record reset time

		"""

		getattr(self, encoder_id + '_reset_time') = time.time()
		getattr(self, encoder_id + '_step_count') = 0

	def set_encoder_direction(self, encoder_id, direction):
		""" Update the stored encoder direction

			:params:
			encoder_id - Unique encoder identifier (string)
			direction  - Encoder's rotation direction. May be integer or string 
		"""

		# Ensure specified encoder exists
		assert encoder_id in self.encoder_ids, "Specified encoder %s does not exist!"%encoder_id

		if isinstance(direction, int):
			assert direction in self.enc_direction_map.keys(), "Specified encoder direction %i is not valid!"%direction
		elif isintance(direction, str):
			allowed_strings = [item for strings in self.enc_direction_map.values() for item in strings]
			assert direction.lower() in allowed_strings, "Specified encoder direction %s is not valid!"%direction
			for key, values in self.enc_direction_map.items():
				if direction.lower() in values:
					direction = key

		getattr(self, encoder_id + '_dir') = direction

	def get_encoder_direction(self, encoder_id):
		""" Return the stored encoder direction

		"""
		return getattr(self, encoder_id + '_dir')

	def get_encoder_reset_time(self, encoder_id):
		""" Return the recorded encoder reset time

		"""
		return getattr(self, encoder_id + '_reset_time')


	def get_encoder_step_count(self, encoder_id):
		""" Return the encoder's current step count

		"""
		return getattr(self, encoder_id, '_step_count')


	def left_up(self, channel):
		""" Wrapper to apply record_encoder_tick during GPIO Interrupt callback
		"""
		self.record_encoder_tick('left_wheel')


	def right_up(self, channel):
		""" Wrapper to apply record_encoder_tick during GPIO Interrupt callback
		"""
		self.record_encoder_tick('right_wheel')


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

		self.last_trig_left = time.time()
		self.last_trig_right = time.time()

		# Setup encoder interrupt callbacks
		## TODO: Generalize this with a lambda call so a unique function is not needed
		#  		 for each encoder 
		GPIO.add_event_detect(self.left_clk, GPIO.RISING, callback= self.left_up, bouncetime= self.bouncetime)
		GPIO.add_event_detect(self.right_clk, GPIO.RISING, callback= self.right_up, bouncetime= self.bouncetime)

		while not rospy.is_shutdown():

			# Calculate measured angular velocity
			for encoder_id in self.encoder_ids:

				# Calculate time since last reset
				d_t = time.time() - get_encoder_reset_time(encoder_id)
				w_meas = self.get_encoder_direction(encoder_id) * self.get_encoder_step_count(encoder_id) * self.enc_step_size / d_t
				print "Encoder %s: Measured angular velocity %0.2f"%(encoder_id, w_meas) 

				# Restart encoder count
				self.reset_encoder(encoder_id)

			rate.sleep()

if __name__ == '__main__':
	try:
		controller = Controller()
		controller.run()
	except rospy.ROSInterruptException:
		GPIO.cleanup()
		pass
