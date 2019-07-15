#!/usr/bin/env python

import rospy
import pygame
import numpy as np

from remote_command.msg import DualshockInputs

AXIS_NAMES = {
	0: 'x_left',
	1: 'y_left',
	2: 'trig_left',
	3: 'x_right', 
	4: 'y_right', 
	5: 'trig_right'
}

def pub_commands():
	## Read inputs from Dualshock Controller & publish to remote_commands message

	pub = rospy.Publisher('remote_commands', DualshockInputs, queue_size=10)	
	rospy.init_node('remote_commands', anonymous=True)

	## Determine if pygame is already initialized
	if not pygame.get_init():
		pygame.init()
	controller = pygame.joystick.Joystick(0)
	controller.init()

	## Initialize at 0 inputs
	inputs = DualshockInputs()

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():

		## Update all joystick values on every joystick event
		for event in pygame.event.get():
			if event.type == pygame.JOYAXISMOTION:
				for axis in AXIS_NAMES.keys():
					setattr(inputs, AXIS_NAMES[axis], controller.get_axis(axis))
				pub.publish(inputs)
		rate.sleep()

if __name__ == '__main__':
	try:
		pub_commands()
	except rospy.ROSInterruptException:
		pass