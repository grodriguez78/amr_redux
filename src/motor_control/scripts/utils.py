#!/usr/bin/env python

w_max = 20 ## Max motor angular velocity (rad/s) 
r_wheel = 1.75 ## Wheel radius (cm)
d_base = 17.8 ## Wheelbase diameter (cm)

V_max = 12 ## Max motor voltage
V_min = 3 ## Min motor voltage to produce movement once static friction is overcome 


def scale_velocity(w):
	""" Convert angular velocity w to a PWM duty cycle 

		Assuming w = 0 at V = V_min and w = w_max at V = V_max, and that 
		angular velocity scales linearily between these values

		param :w: 			The angular velocity to drive a wheel at
		param :V_pwm: 		The voltage to drive  
		return :duty_cycle: An integer for the duty cycle usable by PWM to
							drive a wheel at angular velocity w
	"""

	if w == 0: return 0

	V_pwm = min(abs(float(w))/(w_max), 1)*(V_max - V_min) + V_min
	duty_cycle = (V_pwm / V_max) * 100

	return duty_cycle