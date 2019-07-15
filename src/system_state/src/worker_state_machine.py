#!/usr/bin/env python
import time

import rospy

from system_state.msg import WorkerState

STATE_NAME_MAP= {
	0: 'CHARGING/INACTIVE',
	1: 'MANUAL CONTROL', 
	2: 'AUTONOMOUS CONTROL'
}


def worker_state_machine():
	## Determine the current state of this worker

	## TODO: Name node based on worker ID
	rospy.init_node('lilboi_state_machine', anonymous=True)
	pub = rospy.Publisher('lilboi_state', WorkerState, queue_size=10)

	## Default to state 0 (CHARGING/INACTIVE)
	state = 0
	pub.publish(state)
	state_change_msg = 'Lilboi is now in state %i: (%s)' %(state, STATE_NAME_MAP[state]) 
	rospy.loginfo(state_change_msg)

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():

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
		worker_state_machine()
	except rospy.ROSInterruptException:
		pass