#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from motor_control.msg import WheelVelocity

mcPins = [6, 13, 19, 26] # pins used for motor control

GPIO.setmode(GPIO.BCM)
GPIO.setup(mcPins, GPIO.OUT, initial=GPIO.LOW)

left_fwd = GPIO.PWM(6, 100)
left_bkwds = GPIO.PWM(13, 100)
right_fwd = GPIO.PWM(19, 100)
right_bkwds = GPIO.PWM(26, 100)

def set_wheel_velocities(v_l, v_r):
	## Drive left and right wheels at angular velocities v_l, v_r
	## Using PWM to interpolate between v_max = ?? and v_min = 0


	## Left wheel
	if v_l > 0:
		left_bkwds.stop()
		left_fwd.start(50)

	elif v_l < 0:
		left_fwd.stop()
		left_bkwds.start(50)

	else: 
		left_fwd.stop()
		left_bkwds.stop()

	## Right wheel
	if v_r > 0:
		right_bkwds.stop()
		right_fwd.start(50)

	elif v_r < 0:
		right_fwd.stop()
		right_bkwds.start(50)

	else:
		right_fwd.stop()
		right_bkwds.stop()


def callback(data):
	rospy.loginfo(rospy.get_caller_id() + " Setting right wheel: %d", data.rightVelocity)
	rospy.loginfo(rospy.get_caller_id() + " Setting left wheel: %d", data.leftVelocity)

	set_wheel_velocities(data.leftVelocity, data.rightVelocity)


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