#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO

mcPins = [6, 13, 19, 26] # pins used for motor control

GPIO.setmode(GPIO.BOARD)
GPIO.setup(mcPins, GPIO.OUT, initial=GPIO.LOW)


def controller():

	rospy.init_node('controller', anonymous=True)
	rate = rospy.Rate(10) # 10hz
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