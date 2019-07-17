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

BUTTON_NAMES = {
	0: 'cross', 
	1: 'circle',
	2: 'triangle',
	3: 'square',
	4: 'left_bumper',
	5: 'right_bumper',
	6: 'trig_left',
	7: 'trig_right',
	8: 'share',
	9: 'options',
	10: 'home',
	11: 'left_stick',
	12: 'right_stick'
}

INVERTED_Y_AXIS = True

def pub_commands():
	## Read inputs from Dualshock Controller & publish to remote_commands message

	pub = rospy.Publisher('remote_commands', DualshockInputs, queue_size=10)	
	rospy.init_node('remote_commands', anonymous=True)

	## Determine if pygame is already initialized
	if not pygame.get_init():
		pygame.init()
	controller = pygame.joystick.Joystick(0)
	controller.init()

	## Initialize all inputs to 0
	inputs = DualshockInputs()

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():

		## Update joystick and button values
		for event in pygame.event.get():
			if event.type == pygame.JOYAXISMOTION:
				for axis in AXIS_NAMES.keys():
					if INVERTED_Y_AXIS and axis in [1, 4]:
						setattr(inputs, AXIS_NAMES[axis], -1 * controller.get_axis(axis))
					else:
						setattr(inputs, AXIS_NAMES[axis], controller.get_axis(axis))
				pub.publish(inputs)
			if event.type == pygame.JOYBUTTONDOWN:
				setattr(inputs, BUTTON_NAMES[event.button], True)
				pub.publish(inputs)
			if event.type == pygame.JOYBUTTONUP:
				setattr(inputs, BUTTON_NAMES[event.button], False)
		rate.sleep()


if __name__ == '__main__':
	try:
		pub_commands()
	except rospy.ROSInterruptException:
		pass