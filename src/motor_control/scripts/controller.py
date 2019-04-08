#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from motor_control.msg import WheelVelocity

mcPins = [6, 13, 19, 26] # pins used for motor control

GPIO.setmode(GPIO.BCM)
GPIO.setup(mcPins, GPIO.OUT, initial=GPIO.LOW)


def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "Setting right wheel: %d", data.rightVelocity)
	rospy.loginfo(rospy.get_caller_id() + "Setting left wheel: %d", data.leftVelocity)

def controller():
	## Convert angular velocity signal into individual PWM signals
	rospy.init_node('controller', anonymous=True)

	rospy.Subscriber("robot_dynamics", WheelVelocity, callback)

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