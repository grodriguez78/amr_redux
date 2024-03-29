#!/usr/bin/env python
import time
import rospy
import RPi.GPIO as GPIO


from system_state.msg import WorkerState
from remote_command.msg import DualshockInputs

STATE_NAME_MAP= {
	0: 'CHARGING/INACTIVE',
	1: 'MANUAL CONTROL', 
	2: 'AUTONOMOUS CONTROL'
}

STATE_PINS = {
	0: 22,	## Red
	1: 18,	## Yellow
	2: 17	## Green
}

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(STATE_PINS.values(), GPIO.OUT, initial=GPIO.LOW)

class WorkerStateMachine():

	def __init__(self, state= 0):
		self.state = 0
		self.pub = None
		self.sub = None
		self.triangle = None

		## Set initial state
		self.change_state(state)


	def triangle_pressed(self, triangle_state):
		return not self.triangle and triangle_state


	def display_state(self):

		for pin in STATE_PINS.values():
			GPIO.output(pin, 0)

		GPIO.output(STATE_PINS[self.state], 1)


	def change_state(self, new_state):
		state_change_msg = ' Lilboi transitioned from state %i (%s) to state %i (%s)'%(self.state, STATE_NAME_MAP[self.state], new_state, STATE_NAME_MAP[new_state])
		rospy.loginfo(rospy.get_caller_id() + state_change_msg)
		self.state = new_state
		self.display_state()


	def user_input_transitions(self, data):

		## Charging
		if self.state == 0:
			if self.triangle_pressed(data.triangle): self.change_state(1)

		## Manual Control
		elif self.state == 1:
			if self.triangle_pressed(data.triangle): self.change_state(2)

		## Autonomous Control
		elif self.state == 2:
			if self.triangle_pressed(data.triangle): self.change_state(1)

		## Update local values for next iteration
		self.triangle = data.triangle


	def shutdown(self):
		## Return to state 0

		self.change_state(0)
		self.pub.publish(self.state)


	def run_state_machine(self):
		## Initialize state machine and publish the current state of this worker

		## TODO: Name node based on worker ID
		rospy.init_node('lilboi_state_machine', anonymous=True)
		self.pub = rospy.Publisher('lilboi_state', WorkerState, queue_size=10)
		self.sub = rospy.Subscriber('remote_commands', DualshockInputs, self.user_input_transitions)

		## TODO: Charging auto-transitions

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.pub.publish(self.state)

			rate.sleep()

		self.shutdown()


if __name__ == '__main__':
	try:
		n = WorkerStateMachine()
		n.run_state_machine()
	except rospy.ROSInterruptException:
		GPIO.cleanup()
		pass
