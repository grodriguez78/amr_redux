#!/usr/bin/env python

''' Dummy classes containing hardware constants for access by any module

'''

class motors():
	V_max = 12 ## Max motor voltage
	V_min = 3 ## Min motor voltage to produce movement once static friction is overcome 

	w_max = 20 ## Max motor angular velocity (rad/s) 

class chassis():
	d_base = 0.17 ## Wheelbase diameter (m)

class wheels():
	r = 0.0175 	## Wheel radius (m)
