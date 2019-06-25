#!/usr/bin/env python

''' Dummy classes containing hardware constants for access by any module

'''

class motors():
	V_max = 12 ## Max motor voltage
	V_min = 3 ## Min motor voltage to produce movement once static friction is overcome 

	w_max = 20 ## Max motor angular velocity (rad/s) 


class chassis():
	r_wheel = 1.75 ## Wheel radius (cm)
	d_base = 17.8 ## Wheelbase diameter (cm)
