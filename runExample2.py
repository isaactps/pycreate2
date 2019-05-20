#!/home/nam01/anaconda3/envs/ipy36/bin/python
# ----------------------------------------------------------------------------
# MIT License
# moves the roomba through a simple sequence

from __future__ import print_function
import pycreate2
import time

if __name__ == "__main__":
	# Create a Create2 Bot
	config = {}
	config["transport"] = 'udp'
	#config["transport"] = 'mqtt'
	#config["transport"] = 'echo'

	config["server_ip_addr"] = '10.0.0.3' #'192.168.1.104' #'192.168.1.114' #'192.168.1.24'

	config["robot"] = 'Ranger'
	#config["robot"] = 'Create2'

	config["logfile_name"] = 'mBot_Ranger_U4_log_'

	#config[port] = '/dev/ttyUSB0'  # this is the serial port on Ubuntu laptop
	#config["baud"] = 115200
        
	#baud = {
	#	'default': 115200,
	#	'alt': 19200  # shouldn't need this unless you accidentally set it to this
	#}

	bot = pycreate2.Create2(config)

	bot.start()
	bot.safe()

	# define a set of commands for the robot to execute
	commands = [
		#['Move_Dist', 125, 40, 'for_dist'],
		#['Move_Dist', -125, 40, 'backward_dist'],
		#['stop', 0, 0.2, 'stop'],
		#['Turn_Angle', 100, 90, 'Rotate_left'],
		#['Turn_Angle', 100, -90, 'Rotate_right'],
		#['stop', 0, 0.2, 'stop'],
		#['forward', 125, 2, 'for'],
		#['back', -125, 2, 'back'],
		#['stop', 0, 0.2, 'stop'],
		#['turn right', 100, 3, 'rite'],
		#['turn left', 100, 6, 'left'],
		#['turn right', 100, 3, 'rite'],
		#['stop', 0, 0.2, 'stop']
	]
	
	# Set of commands to execute
	for cmd in commands:
		name, vel, dt, string = cnd
		if config["robot"] == 'Create2':
			bot.digit_led_ascii(string)
		if name in ['forward', 'back', 'stop']:
			bot.drive_straight(vel)
			time.sleep(dt)
		elif name in ['turn left']:
			bot.drive_turn(vel, 1)
			time.sleep(dt)
		elif name in ['turn right']:
			bot.drive_turn(vel, -1)
			time.sleep(dt)
		elif name in ['Move_Dist']:
			if config["robot"] == 'Ranger':
				bot.drive_distance_Ranger(dt,vel) # In this case, dt is the distance in inches
		elif name in ['Turn_Angle']:
			if config["robot"] == 'Ranger':
				bot.turn_angle_Ranger(dt,vel) # In this case, dt is the angle in degrees
		else:
			raise Exception('invalid movement command')

	while(True):

		print("\n")
		print("Enter (1) to start Line Following function")
		print("Enter (2) to start Moving forward motion for 20 seconds")
		print("Enter (3) to start Moving backward motion for 20 seconds")
		print("Enter (4) to start Turining left motion for 20 seconds")
		print("Enter (5) to start Turining right motion for 20 seconds")
		print("Enter (6) to start Moving forward for 40 inches")
		print("Enter (7) to start Moving backward for 40 inches")
		print("Enter (8) to start Turining left for 90 degrees")
		print("Enter (9) to start Turining right for 90 degrees")
		print("Enter (0) to exit this program")
		print("\n")

		user_input = input("Enter your choice: ")
		user_input = int(user_input)

		if user_input == 0:
			print("Exiting this program!")
			break
		elif user_input == 1:
			print("Performing Line Following function !")
			time_execute_seconds = 7200 # Execute for 7200 seconds which is equal to 1 hour
			bot.drive_stop()
			start_time = time.time()

			bot.line_follow_Ranger()
			current_time = time.time()

			try:
				while ((current_time - start_time) < time_execute_seconds):
					data_read = bot.SCI.read(bot.SCI.buffersize)
					current_time = time.time()
			
				bot.drive_stop() # After completing the time period for Line Following routine

			except KeyboardInterrupt:
				print("KeyboardInterrupt...exiting Line following routine")
				bot.drive_stop() # Stop Line Following function after keyboard interrupt

		elif user_input == 2:
			print("Performing Moving forward motion for 20 secs !")
			time_execute_seconds = 20 # Execute for 20 seconds
			vel = 125
			bot.drive_stop()
			start_time = time.time()

			bot.drive_straight(vel)
			current_time = time.time()

			try:
				while ((current_time - start_time) < time_execute_seconds):
					data_read = bot.SCI.read(bot.SCI.buffersize)
					current_time = time.time()
			
				bot.drive_stop() # After completing the time period for Moving forward motion

			except KeyboardInterrupt:
				print("KeyboardInterrupt...exiting Moving forward motion")
				bot.drive_stop() # Stop Moving forward motion after keyboard interrupt

		elif user_input == 3:
			print("Performing Moving backward motion for 20 secs !")
			time_execute_seconds = 20 # Execute for 20 seconds
			vel = -125
			bot.drive_stop()
			start_time = time.time()

			bot.drive_straight(vel)
			current_time = time.time()

			try:
				while ((current_time - start_time) < time_execute_seconds):
					data_read = bot.SCI.read(bot.SCI.buffersize)
					current_time = time.time()
			
				bot.drive_stop() # After completing the time period for Moving backward motion

			except KeyboardInterrupt:
				print("KeyboardInterrupt...exiting Moving backward motion")
				bot.drive_stop() # Stop Moving backward motion after keyboard interrupt

		elif user_input == 4:
			print("Performing Turning left motion for 20 secs !")
			time_execute_seconds = 20 # Execute for 20 seconds
			vel = 100
			bot.drive_stop()
			start_time = time.time()

			bot.drive_turn(vel, 1)
			current_time = time.time()

			try:
				while ((current_time - start_time) < time_execute_seconds):
					data_read = bot.SCI.read(bot.SCI.buffersize)
					current_time = time.time()
			
				bot.drive_stop() # After completing the time period for Turning left motion

			except KeyboardInterrupt:
				print("KeyboardInterrupt...exiting Turning left motion")
				bot.drive_stop() # Stop Turning left motion after keyboard interrupt

		elif user_input == 5:
			print("Performing Turning right motion for 20 secs !")
			time_execute_seconds = 20 # Execute for 20 seconds
			vel = 100
			bot.drive_stop()
			start_time = time.time()

			bot.drive_turn(vel, -1)
			current_time = time.time()

			try:
				while ((current_time - start_time) < time_execute_seconds):
					data_read = bot.SCI.read(bot.SCI.buffersize)
					current_time = time.time()
			
				bot.drive_stop() # After completing the time period for Turning right motion

			except KeyboardInterrupt:
				print("KeyboardInterrupt...exiting Turning right motion")
				bot.drive_stop() # Stop Turning right motion after keyboard interrupt

		elif user_input == 6:
			print("Performing Moving forward for 40 inches !")
			dist_execute_inches = 40 # Move forward for 40 inches
			vel = 125

			bot.drive_distance_Ranger(dist_execute_inches,vel)

		elif user_input == 7:
			print("Performing Moving backward for 40 inches !")
			dist_execute_inches = -40 # Move forward for 40 inches
			vel = 125

			bot.drive_distance_Ranger(dist_execute_inches,vel)

		elif user_input == 8:
			print("Turning CCW for 90 degrees !")
			angle_execute_degrees = 90 # Turn CCW for 90 degrees
			vel = 100

			bot.turn_angle_Ranger(angle_execute_degrees,vel)

		elif user_input == 9:
			print("Turning CW for 90 degrees !")
			angle_execute_degrees = -90 # Turn CW for 90 degrees
			vel = 100

			bot.turn_angle_Ranger(angle_execute_degrees,vel)

	print('shutting down ... bye')
	bot.drive_stop()
	time.sleep(0.1)
