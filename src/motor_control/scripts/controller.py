#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from motor_control.msg import WheelVelocity

mcPins = [6, 13, 19, 26] # pins used for motor control

GPIO.setmode(GPIO.BCM)
GPIO.setup(mcPins, GPIO.OUT, initial=GPIO.LOW)

left_fwd = GPIO.PWM(19, 100)
left_bkwds = GPIO.PWM(26, 100)
right_fwd = GPIO.PWM(6, 100)
right_bkwds = GPIO.PWM(13, 100)

w_max = 20 ## Max motor angular velocity (rad/s) 
r_wheel = 1.75 ## Wheel radius (cm)
d_base = 17.8 ## Wheelbase diameter (cm)


def scale_velocity(w):
	""" Convert angular velocity w to a PWM duty cycle 

	"""
	return min(abs(w)/w_max, 1)*100


def set_wheel_velocities(w_left, w_right):
	""" Drive left and right wheels at angular velocities w_l, w_r

	"""

	## Left wheel
	if v_left > 0:
		left_bkwds.stop()
		left_fwd.start(scale_velocity(w_left))

	elif v_left < 0:
		left_fwd.stop()
		left_bkwds.start(scale_velocity(w_left))

	else: 
		left_fwd.stop()
		left_bkwds.stop()

	## Right wheel
	if v_right > 0:
		right_bkwds.stop()
		right_fwd.start(scale_velocity(w_right))

	elif v_right < 0:
		right_fwd.stop()
		right_bkwds.start(scale_velocity(w_right))

	else:
		right_fwd.stop()
		right_bkwds.stop()


def callback(data):
	rospy.loginfo(rospy.get_caller_id() + " Setting right wheel: %d", data.w_left)
	rospy.loginfo(rospy.get_caller_id() + " Setting left wheel: %d", data.w_right)

	set_wheel_velocities(data.w_left, data.w_right)


def controller():
	## Convert angular velocity signal into individual PWM signals
	rospy.init_node('controller', anonymous=True)

	s = rospy.Subscriber("robot_dynamics", WheelVelocity, callback)

	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		# hello_str = "Hello world!"
		# rospy.loginfo(hello_str)
		rate.sleep()

if __name__ == '__main__':
	try:
		controller()
	except rospy.ROSInterruptException:
		GPIO.cleanup()
		pass
