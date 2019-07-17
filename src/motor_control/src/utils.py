#!/usr/bin/env python

from hardware_config import motors

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

	V_pwm = min(abs(float(w))/(motors.w_max), 1)*(motors.V_max - motors.V_min) + motors.V_min
	duty_cycle = (V_pwm / motors.V_max) * 100

	return duty_cycle