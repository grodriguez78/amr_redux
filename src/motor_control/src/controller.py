#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from motor_control.msg import WheelVelocity

from utils import scale_velocity

mcPins = [16, 20, 19, 26] # pins used for motor control

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(mcPins, GPIO.OUT, initial=GPIO.LOW)

left_fwd = GPIO.PWM(26, 100)
left_bkwds = GPIO.PWM(19, 100)
right_fwd = GPIO.PWM(16, 100)
right_bkwds = GPIO.PWM(20, 100)


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

	while not rospy.is_shutdown():

		rate.sleep()

if __name__ == '__main__':
	try:
		controller()
	except rospy.ROSInterruptException:
		GPIO.cleanup()
		pass
