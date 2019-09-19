#!/usr/bin/env python

import rospy
import time
import numpy as np

import RPi.GPIO as GPIO

from motor_control.msg import WheelVelocity
from utils import scale_velocity

mcPins = [16, 20, 19, 26] 	# GPIO pins used for motor control
encPins = [13, 6, 24, 23] 	# GPIO pins used to read encoders

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(mcPins, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(encPins, GPIO.IN)

## Control pins
left_fwd = GPIO.PWM(26, 100)
left_bkwds = GPIO.PWM(19, 100)
right_fwd = GPIO.PWM(16, 100)
right_bkwds = GPIO.PWM(20, 100)

## Encoder pins
left_clk = 6
left_dt = 13
right_clk = 23
right_dt = 24

## Storage for encoder trigger times
LAST_TRIG_LEFT = 0.0
LAST_TRIG_RIGHT = 0.0

## Angular distance of a single encoder step, in radians
KY_040_STEP = 2 * np.pi / 30

def encoder_direction(clk, dt):
	""" Determine encoder direction from the current state of clk and dt pins
		
		clk 	pin number for an encoder's clk output
		dt 		pin number for an encoder's dt output

		return   1 - clockwise direction
			    -1 - counter-clockwise direction
	"""

	# Wheel is rotating clockwise
	if GPIO.input(left_clk) != GPIO.input(left_dt):
		direction = 1

	# Wheel is rotating counter-clockwise
	else:
		direction = -1

	return direction

def left_up(channel):
	""" Note the time the left encoder's clk went high and determine which
		direction the left wheel is turning
	"""

	# Record trigger time
	t_trig = time.time()

	# Determine rotation direction
	direction = encoder_direction(left_clk, left_dt)

	# Calculate angular velocity
	d_t = t_trig - LAST_TRIG_LEFT
	w = direction * KY_040_STEP / d_t

	# Reset recorded trigger time
	LAST_TRIG_LEFT = t_trig

	print "Left Wheel Angular Velocity: %0.2f"%w


def right_up(channel):
	""" Note the time the right encoder's clk went high and determine which
		direction the wheel is turning
	"""

	# Record trigger time
	t_trig = time.time()

	# Determine rotation direction
	direction = encoder_direction(right_clk, right_dt)

	# Calculate angular velocity
	d_t = t_trig - LAST_TRIG_RIGHT
	w = direction * KY_040_STEP / d_t

	# Reset recorded trigger time
	LAST_TRIG_RIGHT = t_trig

	print "Right Wheel Angular Velocity: %0.2f"%w


def set_wheel_velocities(w_left, w_right):
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


def callback(data):
	rospy.loginfo(rospy.get_caller_id() + " w_left: %d", data.w_left)
	rospy.loginfo(rospy.get_caller_id() + " w_right: %d", data.w_right)

	set_wheel_velocities(data.w_left, data.w_right)


def controller():
	## Convert angular velocity signal into individual PWM signals
	rospy.init_node('controller', anonymous=True)

	s = rospy.Subscriber("robot_dynamics", WheelVelocity, callback)

	rate = rospy.Rate(10) # 10hz

	LAST_TRIG_LEFT = time.time()
	LAST_TRIG_RIGHT = time.time()

	while not rospy.is_shutdown():

		rate.sleep()

if __name__ == '__main__':
	try:
		controller()
	except rospy.ROSInterruptException:
		GPIO.cleanup()
		pass
