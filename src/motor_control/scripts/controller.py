#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from motor_control.msg import WheelVelocity

mcPins = [6, 13, 19, 26] # pins used for motor control

GPIO.setmode(GPIO.BCM)
GPIO.setup(mcPins, GPIO.OUT, initial=GPIO.LOW)

left_fwd = GPIO.PWM(6, 100)
left_bkwds = GPIO.PWM(13, 100)


def callback(data):
	rospy.loginfo(rospy.get_caller_id() + " Setting right wheel: %d", data.rightVelocity)
	rospy.loginfo(rospy.get_caller_id() + " Setting left wheel: %d", data.leftVelocity)

	if data.leftVelocity > 0:
		left_bkwds.stop()
		left_fwd.start(50)

	elif data.leftVelocity < 0:
		left_fwd.stop()
		left_bkwds.start(50)

	else: 
		left_fwd.stop()
		left_bkwds.stop()


def controller():
	## Convert angular velocity signal into individual PWM signals
	rospy.init_node('controller', anonymous=True)

	s = rospy.Subscriber("robot_dynamics", WheelVelocity, callback)

	rate = rospy.Rate(10) # 10hz


	# Set pins 6, 19 to HIGH
	GPIO.output(6, 1)
	GPIO.output(19, 1)
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