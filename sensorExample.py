#!/usr/bin/env python
# ----------------------------------------------------------------------------
# MIT License
# shows how to get sensor data from the create 2

from __future__ import print_function
from pycreate2 import Create2
import time


if __name__ == "__main__":
        config = {}
        config["transport"] = 'echo'
	bot = Create2(config)

	bot.start()

	bot.safe()

	print('Starting ...')

	while True:
		# Packet 100 contains all sensor data.
                sensor_state = ''
                try:
                    sensor_state = bot.get_sensors()
                except:
                    pass

		print('==============Updated Sensors==============')
		print(sensor_state)
		time.sleep(2)
