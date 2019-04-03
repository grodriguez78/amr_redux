#!/usr/bin/env python

import rospy
from motor_control.msg import WheelVelocity

def motionPlanner():
	## Convert intertial v_x, v_y, eps into 
	## individual wheel angular velocities

	pub = rospy.Publisher('robot_dynamics', WheelVelocity, queue_size=10)
	rospy.init_node('motionPlanner', anonymous=True)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		rospy.loginfo('woooo')
		pub.publish(10, 5)
		rate.sleep()

if __name__ == '__main__':
	try:
		motionPlanner()
	except rospy.ROSInterruptException:
		pass