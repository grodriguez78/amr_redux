#!/usr/bin/env python

""" Encoders.py

	This node processes encoder signals and publishes MeasuredAngularVelocity
	messages to the /robot_dynamics/ topic
"""

import os
import time

import yaml
import rospy
import numpy as np
import RPi.GPIO as GPIO

import hardware_config
from odometry.msg import EncoderValue


class Encoders(object):

	def __init__(self, encoder_params):

		# Encoder identifiers
		self.encoder_ids = encoder_params['instances'].keys()

		# GPIO setup
		self.save_pin_info(encoder_params, self.encoder_ids)
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.encoder_pins, GPIO.IN)

		# Angular distance of a single encoder step, in radians
		self.enc_step_size = encoder_params['specifications']['step_size']

		# Calculate bouncetime (ms) from max angular velocity
		self.bouncetime = int(1000 * 0.5 * self.enc_step_size / 20)

		# Initialize storage attributes
		for encoder_id in self.encoder_ids:
			setattr(self, encoder_id + '_step_count', 0)	# Encoder step count
			setattr(self, encoder_id + '_dir', 0)			# Encoder direction
			setattr(self, encoder_id + '_reset_time', None) # Encoder reset time

		# Accepted string & int values for encoder directions
		self.enc_direction_map = {
			-1: ['ccw', 'counter-clockwise'],
			0:  ['stopped'],
			1:  ['cw', 'clockwise']
		}

		# Wheel velocities
		self.left_wheel_meas_w = 0.0
		self.right_wheel_meas_w = 0.0


	def save_pin_info(self, encoder_params, encoder_ids):
		""" Create attributes to store GPIO pin numbers according 
			to pin names in encoder params
		"""

		# Setup GPIO pins
		encoder_pins = []
		for encoder_id in encoder_ids:
			pins = encoder_params['instances'][encoder_id]['gpio_pins']
			encoder_pins += pins.values()

			# Save pins
			for pin_name, pin in pins.items(): setattr(self, encoder_id + '_' + pin_name, pin)
		self.encoder_pins = encoder_pins 


	def reset_encoder(self, encoder_id):
		""" Set encoder step count to 0 and record reset time

		"""
		setattr(self, encoder_id + '_reset_time', time.time())
		setattr(self, encoder_id + '_step_count', 0)


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
		return getattr(self, encoder_id + '_step_count')


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

		setattr(self, encoder_id + '_dir', direction)


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


	def left_up(self, channel):
		""" Wrapper to apply record_encoder_tick during GPIO Interrupt callback
		"""
		self.record_encoder_tick('left_wheel')


	def right_up(self, channel):
		""" Wrapper to apply record_encoder_tick during GPIO Interrupt callback
		"""
		self.record_encoder_tick('right_wheel')

	def run(self):

		rospy.init_node('encoders', anonymous= True)

		rate = rospy.Rate(10) # 10 Hz

		# Initialize encoder reset times
		for encoder_id in self.encoder_ids:
			self.reset_encoder(encoder_id)

		# Setup encoder interrupt callbacks
		## TODO: Generalize this with a lambda call so a unique function is not needed
		#  		 for each encoder (https://www.raspberrypi.org/forums/viewtopic.php?t=96622)
		GPIO.add_event_detect(self.left_wheel_clk, GPIO.RISING, callback= self.left_up, bouncetime= self.bouncetime)
		GPIO.add_event_detect(self.right_wheel_clk, GPIO.RISING, callback= self.right_up, bouncetime= self.bouncetime)

		while not rospy.is_shutdown():

			# Calculate measured angular velocity
			for encoder_id in self.encoder_ids:

				# Calculate time since last reset
				d_t = time.time() - self.get_encoder_reset_time(encoder_id)
				w_meas = self.get_encoder_direction(encoder_id) * self.get_encoder_step_count(encoder_id) * self.enc_step_size / d_t
				print "Encoder %s: Measured angular velocity %0.2f"%(encoder_id, w_meas) 

				# Restart encoder count
				self.reset_encoder(encoder_id)

			rate.sleep()


if __name__ == '__main__':
	try:

		# Load encoder configuration
		sensor_config_file = hardware_config.__path__[-1] + '/sensor_config.yaml'
		with open(sensor_config_file) as stream:
			sensor_configs = yaml.load(stream)
		encoder_params = sensor_configs['encoders']

		encoders = Encoders(encoder_params)
		encoders.run()

	except rospy.ROSInterruptException:
		GPIO.cleanup()
		pass
