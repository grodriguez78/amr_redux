#!/usr/bin/env python

import rospy

from motor_control.msg import WheelVelocity
from system_state.msg import WorkerState
from remote_command.msg import DualshockInputs
from hardware_config import motors

class Planner():

	def __init__(self):
		self.state = None
		self.cmd_right = 0
		self.cmd_left = 0


	def rem_cmd_callback(self, data):
		## Convert joystick inputs to wheel commands 

		if self.state == 0 or self.state == 1:
			self.cmd_right = motors.w_max * data.y_right
			self.cmd_left = motors.w_max * data.y_left
		
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

			pub.publish(self.cmd_left, self.cmd_right)
			rate.sleep()

if __name__ == '__main__':
	try:
		planner = Planner()
		planner.plan_motion()
	except rospy.ROSInterruptException:
		pass