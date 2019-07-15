#!/usr/bin/env python

import rospy
from motor_control.msg import WheelVelocity
from remote_commands.msg import DualshockInputs


class Planner():

	def __init__(self):
		self.state = None
		self.cmd_right = 0
		self.cmd_left = 0


	def cmd_callback(data):
		if self.state == 0:
			import pdb; pdb.set_trace()


	def state_callback(data):
		self.state = data.state


	def plan_motion():
		## Convert intertial v_x, v_y, eps into 
		## individual wheel angular velocities

		pub = rospy.Publisher('robot_dynamics', WheelVelocity, queue_size=10)
		state_sub = rospy.Subscriber('lilboi_state', WorkerState, state_callback)
		cmd_sub = rospy.Subscriber('remote_commands', DualshockInputs, cmd_callback)

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