#!/usr/bin/env python

"""Ping BigBoi to determine wifi status and display on WifiLED.

"""

import logging
import os
import time

import yaml

import hardware_config

# Setup logging to repo's /logs directory
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

f_handler = logging.FileHandler(os.environ["HOME"] + '/amr_redux/logs/wifi_status.log')
f_handler.setLevel(logging.INFO)

f_format = logging.Formatter("%(asctime)s %(name)s %(levelname)s: %(message)s")
f_handler.setFormatter(f_format)

logger.addHandler(f_handler)

def ping_ros_master():
	"""Ping the ROS master and return result.

	Returns
	-------
	resp - int
		0 if success
	"""
	return os.system("ping -c 1 " + os.environ["ROS_HOSTNAME"])


def update_led(status):
	"""Set the LED based on current WiFi status.

	Parameters
	----------
	status - bool
		True if connected to ros master.
	"""

	if status:
		logger.info("Connected!")
	else:
		logger.info("Disconnected!")


def status_from_response(resp):
	"""Determine connectivity status from ping response code.

	Parameters
	----------
	resp - Int
		0  - Connected
		!0 - Disconnected

	Return
	------
	status - Bool
		True if connected to master.
	"""
	status = False
	if resp == 0: status = True
	return status


def main(freq= 1.0):
	"""Ping ROS_MASTER (BigBoi) and display wifi status.

	Checks with frequency = 1Hz by default.
	Updates wifi LED only when connection is lost or established.

	Parameters
	----------
	freq - float
		Frequency to check connectivity.
	"""

	# Get wifi pin # from display configuration parameters
	display_config_file = hardware_config.__path__[-1] + '/display_config.yaml'
	with open(display_config_file) as stream:
		display_hardware_params = yaml.load(stream) 
	wifi_pin = display_hardware_params['leds']['wifi']
	logger.info("Displaying WiFi status on GPIO Pin %i"%wifi_pin)

	# Turn off LED by default
	last_status = False
	update_led(last_status)

	# Continuously check for status
	while True:
		resp = ping_ros_master()
		status = status_from_response(resp)

		# Update if status is changed
		if status != last_status: 
			update_led(status)
			last_status = status

		# Go to sleep until next check
		time.sleep(freq)

if __name__ == "__main__":
	main()