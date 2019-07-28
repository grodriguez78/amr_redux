#!/usr/bin/env python

import rospy
import numpy as np

from motor_control.msg import WheelVelocity
from system_state.msg import WorkerState
from remote_command.msg import DualshockInputs
from hardware_config import motors, wheels, chassis

class Planner():

	def __init__(self):
		self.state = None
		self.cmd_right = 0
		self.cmd_left = 0

		""" Forward kinematic model for manual control
			
			[w_r, w_l]^-1 = inv(K_man) [v_x, psi]

		"""
		self.K_man = (wheels.r / 2) * np.array([[1, 1],[2 / chassis.d_base, -2 / chassis.d_base]])

		self.max_v = 0.25
		self.max_w = 1.75


	def rem_cmd_callback(self, data):
		## Convert joystick inputs to wheel commands 

		## Manual Control, convert joystick axis to robot v, w
		if self.state == 1:

			## Convert joystick values to m/s, rad/s values
			v = data.y_left * self.max_v
			w_cmd = -1.0 * data.x_right * self.max_w

			cmds = np.matmul(np.linalg.inv(self.K_man), np.array([v, w_cmd]))

			self.cmd_right = cmds[0]
			self.cmd_left = cmds[1]
		
		## TODO: Remove this once autonomous commands are implemented
		else:
			self.cmd_right = 0
			self.cmd_left = 0


	def state_callback(self, data):
		self.state = data.state


	def plan_motion(self):
		## Convert intertial v_x, v_y, eps into 
		## individual wheel angular velocities

		pub = rospy.Publisher('robot_dynamics', WheelVelocity, queue_size=10)
		state_sub = rospy.Subscriber('lilboi_state', WorkerState, self.state_callback)
		cmd_sub = rospy.Subscriber('remote_commands', DualshockInputs, self.rem_cmd_callback)

		rospy.init_node('motionPlanner', anonymous=True)
		rate = rospy.Rate(10)

		foo = rospy.get_param_names()

		while not rospy.is_shutdown():

			pub.publish(self.cmd_right, self.cmd_left)
			rate.sleep()

if __name__ == '__main__':
	try:
		planner = Planner()
		planner.plan_motion()
	except rospy.ROSInterruptException:
		pass