#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO

mcPins = [6, 13, 19, 26] # pins used for motor control

GPIO.setmode(GPIO.BCM)
GPIO.setup(mcPins, GPIO.OUT, initial=GPIO.LOW)


def controller():
	## Convert angular velocity signal into individual PWM signals
	rospy.init_node('controller', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	# Set pins 6, 19 to HIGH
	GPIO.output(6, 1)
	GPIO.output(19, 1)
	while not rospy.is_shutdown():
		hello_str = "Hello world!"
		rospy.loginfo(hello_str)
		rate.sleep()

if __name__ == '__main__':
	try:
		controller()
	except rospy.ROSInterruptException:
		GPIO.cleanup()
		pass