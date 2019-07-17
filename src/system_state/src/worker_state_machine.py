#!/usr/bin/env python
import time

import rospy

from system_state.msg import WorkerState
from remote_command.msg import DualshockInputs

STATE_NAME_MAP= {
	0: 'CHARGING/INACTIVE',
	1: 'MANUAL CONTROL', 
	2: 'AUTONOMOUS CONTROL'
}

class WorkerStateMachine():

	def __init__(self):
		self.state = 0
		self.pub = None
		self.sub = None
		self.triangle = None


	def triangle_pressed(self, triangle_state):
		return not self.triangle and triangle_state


	def change_state(self, new_state):
		state_change_msg = 'Lilboi transitioned from state %i (%s) to state %i (%s)'%(self.state, STATE_NAME_MAP[self.state], new_state, STATE_NAME_MAP[new_state])
		rospy.loginfo(state_change_msg)
		self.state = new_state


	def user_input_transitions(self, data):

		## Charging
		if self.state == 0:
			if self.triangle_pressed(data.triangle): self.change_state(1)

		elif self.state == 1:
			if self.triangle_pressed(data.triangle): self.change_state(2)

		elif self.state == 2:
			if self.triangle_pressed(data.triangle): self.change_state(1)

		## Update local values for next iteration
		self.triangle = data.triangle


	def run_state_machine(self):
		## Determine the current state of this worker

		## TODO: Name node based on worker ID
		rospy.init_node('lilboi_state_machine', anonymous=True)
		self.pub = rospy.Publisher('lilboi_state', WorkerState, queue_size=10)
		self.sub = rospy.Subscriber('remote_commands', DualshockInputs, self.user_input_transitions)

		## Default to state 0 (CHARGING/INACTIVE)
		state = 0
		state_change_msg = 'Lilboi is now in state %i: (%s)' %(state, STATE_NAME_MAP[state]) 
		rospy.loginfo(state_change_msg)

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.pub.publish(self.state)

			# if state == 0:
			# 	if triangle is pressed: state = 1 
			# 	if charged and task is available: state = 2

			# elif state == 1:
			# 	if battery is depleted: state = 0
			#	if triangle is pressed: state = 2

			# elif state == 2:
			# 	if battery is depleted: state = 0
			# 	if trangle is pressed: state = 1


			rate.sleep()

if __name__ == '__main__':
	try:
		n = WorkerStateMachine()
		n.run_state_machine()
	except rospy.ROSInterruptException:
		pass