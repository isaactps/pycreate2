# The MIT License
#
# Copyright (c) 2007 Damon Kohler
# Copyright (c) 2015 Jonathan Le Roux (Modifications for Create 2)
# Copyright (c) 2015 Brandon Pomeroy
# Copyright (c) 2017 Kevin Walchko
#
# This is the main code for interacting with the Create 2

from __future__ import print_function
from __future__ import division
import struct  # there are 2 places that use this ... why?
import time
from pycreate2.packets import SensorPacketDecoder
from pycreate2.packets import SensorPacketDecoder_0
from pycreate2.packets import SensorPacketDecoder_101

from pycreate2.createSerial import SerialCommandInterface
from pycreate2.createMQTT import MQTTCommandInterface
from pycreate2.createTCP import TCPCommandInterface
from pycreate2.createUDP import UDPCommandInterface
from pycreate2.createEcho import EchoCommandInterface
from pycreate2.OI import OPCODES
# from pycreate2.OI import calc_query_data_len
from pycreate2.OI import DRIVE


# Not sure of the value of these Yet
# class Fatal(Exception):
# 	pass
#
#
# class Error(Exception):
# 	pass
#
#
# class Warning(Exception):
# 	pass

#Constants for iRobot Create 2 platform
WHEEL_PERIMETER = 226.19    # wheel perimeter in mm
WHEEL_COUNTS = 511.0        # encoder counts per full revolution
DRIVE_PERIMETER = 738.2742  # 2*pi*115.4; 115.4 = Create2 centre to wheel centre
CLIFF_THRES = 100           # threshold for cliff sensors to detect a cliff
DRIVE_SPEED = 80            # suitable speed for driving straight
TURN_SPEED = 70             # suitable speed for turning (on the spot)

class Create2(object):
	"""
	The top level class for controlling a Create2.
	This is the only class that outside scripts should be interacting with.
	"""

	def __init__(self, config):
		"""
		Constructor, sets up class
		- creates serial port
		- creates decoder
		"""
		if config["transport"] == 'mqtt':
			self.SCI = MQTTCommandInterface()
			self.SCI.open(port)
		elif config["transport"] == 'echo':
			self.SCI = EchoCommandInterface()
			self.SCI.open()
		elif config["transport"] == 'tcp':
			self.SCI = TCPCommandInterface()
			self.SCI.open(config["server_ip_addr"])
		elif config["transport"] == 'udp':
			self.SCI = UDPCommandInterface()
			self.SCI.open(config["server_ip_addr"],config["logfile_name"])
		else:
			self.SCI = SerialCommandInterface()
			self.SCI.open(config["port"], config["baud"])

		# self.decoder = SensorPacketDecoder()
		self.decoder = None
		self.sleep_timer = 0.5

		if config["robot"] == 'Create2':
			self.robot = 'Create2'
		elif config["robot"] == 'Ranger':
			self.robot = 'Ranger'

	#def __init__(self, port, baud=115200):
		"""
		Constructor, sets up class
		- creates serial port
		- creates decoder
		"""
		"""
		if 'mqtt' in port:
			self.SCI = MQTTCommandInterface()
			self.SCI.open(port)
		elif 'tcp' in port:
			self.SCI = TCPCommandInterface()
			self.SCI.open(port)
		else:
			self.SCI = SerialCommandInterface()
			self.SCI.open(port, baud)
		# self.decoder = SensorPacketDecoder()
		self.decoder = None
		self.sleep_timer = 0.5
		"""

	def __del__(self):
		"""Destructor, cleans up when class goes out of scope"""

		if self.robot == 'Create2':
			# stop motors
			self.drive_stop()
			time.sleep(self.sleep_timer)

			# turn off LEDs
			self.led()
			self.digit_led_ascii('    ')
			time.sleep(0.1)

			# close it down
			# self.power()
			# self.safe()  # this beeps every now and then, but doesn't seem to go off
			time.sleep(0.1)
			self.stop()  # power down, makes a low beep sound
			time.sleep(0.1)

		self.close()  # close serial port
		time.sleep(0.1)

	def close(self):
		"""
		Closes up serial ports and terminates connection to the Create2
		"""
		self.SCI.close()

	# ------------------- Mode Control ------------------------

	def start(self):
		"""
		Puts the Create 2 into Passive mode. You must always send the Start command
		before sending any other commands to the OI.
		"""
		# self.SCI.open()
		self.SCI.write(OPCODES.START)
		time.sleep(self.sleep_timer)

	def getMode(self):
		"""
		This doesn't seem to work
		"""
		self.SCI.write(OPCODES.MODE)
		time.sleep(0.005)
		ans = self.SCI.read(1)
		if len(ans) == 1:
			byte = struct.unpack('B', ans)[0]
		else:
			byte = 'Error, not mode returned'
		print('Mode: {}'.format(byte))

	def wake(self):
		"""
		Wake up robot. See OI spec, pg 7 under passive mode. This should reset
		the 5 min timer in passive mode.

		Unfortunately, if you are using the "offical" create cable ... it doesn't
		work! They wired it wrong:
		https://robotics.stackexchange.com/questions/7895/irobot-create-2-powering-up-after-sleep
		"""
		self.SCI.ser.rts = True
		self.SCI.ser.dtr = True
		time.sleep(1)
		self.SCI.ser.rts = False
		self.SCI.ser.dtr = False
		time.sleep(1)
		self.SCI.ser.rts = True
		self.SCI.ser.dtr = True
		time.sleep(1)  # Technically it should wake after 500ms.

	def reset(self):
		"""
		This command resets the robot, as if you had removed and reinserted the
		battery.

		('Firmware Version:', 'bl-start\r\nSTR730\r\nbootloader id: #x47186549 82ECCFFF\r\nbootloader info rev: #xF000\r\nbootloader rev: #x0001\r\n2007-05-14-1715-L   \r')
		"""
		self.SCI.write(OPCODES.RESET)
		time.sleep(1)
		ret = self.SCI.read(128)
		return ret

	def stop(self):
		"""
		Puts the Create 2 into OFF mode. All streams will stop and the robot will no
		longer respond to commands. Use this command when you are finished
		working with the robot.
		"""
		self.SCI.write(OPCODES.STOP)
		time.sleep(self.sleep_timer)

	def safe(self):
		"""
		Puts the Create 2 into safe mode. Blocks for a short (<.5 sec) amount
		of time so the bot has time to change modes.
		"""
		self.SCI.write(OPCODES.SAFE)
		time.sleep(self.sleep_timer)

	def full(self):
		"""
		Puts the Create 2 into full mode. Blocks for a short (<.5 sec) amount
		of time so the bot has time to change modes.
		"""
		self.SCI.write(OPCODES.FULL)
		time.sleep(self.sleep_timer)

	# def seek_dock(self):
	# 	self.SCI.write(OPCODES.SEEK_DOCK)

	def power(self):
		"""
		Puts the Create 2 into Passive mode. The OI can be in Safe, or
		Full mode to accept this command.
		"""
		self.SCI.write(OPCODES.POWER)
		time.sleep(self.sleep_timer)

	# ------------------ Drive Commands ------------------

	def drive_stop(self):
		self.drive_straight(0)
		time.sleep(self.sleep_timer)  # wait just a little for the robot to stop

	def drive_rotate(self, velocity, direction):
		"""
		Turn in place clockwise: -1 CW
		Turn in place counterclockwise: 1 CCW
		"""
		if direction in [DRIVE.TURN_CW, DRIVE.TURN_CCW]:
			pass
		else:
			raise Exception('Create2::drive_rotate() invalid direction:', direction)

		v = self.limit(velocity, -500, 500)
		# print('drive_rotate: {} {}'.format(v, direction))
		data = struct.unpack('4B', struct.pack('>2h', v, direction))
		self.SCI.write(OPCODES.DRIVE, data)

	def drive_turn(self, velocity, radius):
		"""
		The create will turn

		velocity: [-500,500] speed in mm/s
		radius: [-2000,2000] radius of the turn in mm
		"""
		v = self.limit(velocity, -500, 500)
		r = self.limit(radius, -2000, 2000)
		#print('drive_turn: {} {}'.format(v, r))
		data = struct.unpack('4B', struct.pack('>2h', v, r))
		self.SCI.write(OPCODES.DRIVE, data)

	def drive_straight(self, velocity):
		"""
		Will make the Create2 drive straight at the given velocity

		Arguments:
			velocity: Velocity of the Create2 in mm/s. Positive velocities are forward,
				negative velocities are reverse. Max speeds are still enforced by drive()

		"""
		v = self.limit(velocity, -500, 500)
		#print('drive_straight: {}'.format(v))
		data = struct.unpack('4B', struct.pack('>hH', v, DRIVE.STRAIGHT))  # write do this?
		self.SCI.write(OPCODES.DRIVE, data)

	def limit(self, val, low, hi):
		val = val if val < hi else hi
		val = val if val > low else low
		return val

	def drive_direct(self, r_vel, l_vel):
		"""
		Drive motors directly: [-500, 500] mm/sec
		"""
		r_vel = self.limit(r_vel, -500, 500)
		l_vel = self.limit(l_vel, -500, 500)
		data = struct.unpack('4B', struct.pack('>2h', r_vel, l_vel))  # write do this?
		self.SCI.write(OPCODES.DRIVE_DIRECT, data)

	def drive_pwm(self, r_pwm, l_pwm):
		"""
		Drive motors PWM directly: [-255, 255] PWM
		"""
		r_pwm = self.limit(r_pwm, -255, 255)
		l_pwm = self.limit(l_pwm, -255, 255)
		data = struct.unpack('4B', struct.pack('>2h', r_pwm, l_pwm))  # write do this?
		self.SCI.write(OPCODES.DRIVE_PWM, data)

	def turn_angle(self, angle, speed=100):
		"""
		Uses the encoders to turn an angle in degrees. This is a best effort,
		the results will not be perfect due to wheel slip and encoder errors.

		CCW is positive
		CW is negative
		"""
		turn_angle = 0.0

		if angle > 0:
			cmd = (speed, -speed)  # R, L
		else:
			cmd = (-speed, speed)
			angle = -angle

		while abs(turn_angle) < angle:
			self.drive_direct(*cmd)
			sensors = self.get_sensors()
			turn_angle += sensors.angle  # roomba only tracks the delta angle

	def drive_distance(self, distance, speed=100, stop=False):
		"""
		Uses the encoders to drive straight forward or backwards. This is a best effort,
		the results will not be perfect due to wheel slip and encoder errors.
			distance: meters
			speed: mm/sec, positive is forward, negative is backwards
		"""
		mov = 0.0

		if distance > 0:
			cmd = (speed, speed)  # R, L
		else:
			cmd = (-speed, -speed)
			distance = -distance

		while abs(mov) < distance:
			self.drive_direct(*cmd)
			sensors = self.get_sensors()
			mov += sensors.distance/1000  # roomba only tracks the delta distance

		if stop:
			self.drive_stop()

	def turn_angle_v2(self, angle, speed=TURN_SPEED): #Alternative version to command the iRobot to rotate by a specified angle
		"""
		Uses the encoders to turn an angle in degrees. This is a best effort,
		the results will not be perfect due to wheel slip and encoder errors.

		CCW is positive
		CW is negative
		"""

		arclength = (float(abs(angle))/360.0) * DRIVE_PERIMETER 
		
		#since we are only reading from left encoder in driveDistance()
		if angle > 0:
		    arclength *= -1
		else:
		    arclength = arclength
		
		#print("arc_length: " + str(arclength))
		if angle < 0:
		    self.drive_distance_v2(speed, (-1)*speed, arclength, 0) 
		else:
		    self.drive_distance_v2((-1)*speed, speed, arclength, 0)

		# stop Robot
		self.drive_stop()

		return 0

	def drive_distance_v2(self, l_vel, r_vel, distance, if_straight): #Alternative version to command the iRobot to move by a specified distance
		"""
		Uses the encoders to drive forward or backwards in either straight or turning manner. This is a best effort,
		the results will not be perfect due to wheel slip and encoder errors.
		distance: mm
		velocity: mm/sec, positive is forward, negative is backwards
		"""
		
		#print("des_dis: " + str(distance))

		count0 = self.getEncoderCountL()     #only use the left encoder
		#print("count0: " + str(count0))

		if if_straight == 1:
		    if distance < 0:
		        self.drive_direct(-r_vel, -r_vel) # right wheel velocity, left wheel velocity
			#self.drive_straight(-r_vel)
		    else:
		        self.drive_direct(r_vel, r_vel)
			#self.drive_straight(r_vel)
		else:
		    self.drive_direct(r_vel, l_vel)

		count1 = self.getEncoderCountL()

		#print("count1: " + str(count1))

		cur_distance = ( ((count1-count0)/WHEEL_COUNTS)*WHEEL_PERIMETER ) 
		#print("cur_dis: " + str(cur_distance))
		prev_count1 = count1

		#keep refreshing distance while robot drives
		while abs(cur_distance) < abs(distance-1):
			prev_count1 = count1
			count1 = self.getEncoderCountL()
			#print("count1: " + str(count1))

			if (distance > 0) and (count1 < count0) and (abs(count1 - prev_count1) > 65000): # rollover from 65535 to 0 has occurred
				count1 += 65535

			elif (distance < 0) and (count1 > count0) and (abs(count1 - prev_count1) > 65000): # rollover from 0 to 65535 has occurred
				count1 -= 65535	

			cur_distance = ((count1-count0)/WHEEL_COUNTS)*WHEEL_PERIMETER 
			#print("cur_dis: " + str(cur_distance))
			time.sleep(0.015)

		#print("cur_dis at End: " + str(cur_distance))
		#print("des_dis at End: " + str(distance))

		# stop Robot
		self.drive_stop()

		return 0

	def turn_angle_Ranger(self, angle, speed=150):
		"""
		Uses the encoders to turn an angle in degrees. This is a best effort,
		the results will not be perfect due to wheel slip and encoder errors.

		CCW is positive
		CW is negative
		"""

		speed = self.limit(speed, 0, 255)
		data = struct.unpack('4B', struct.pack('>2h', speed, angle))
		self.SCI.write(OPCODES.TURN_ANGLE_RANGER, data)

	def drive_distance_Ranger(self, distance, speed=150):
		"""
		Uses the encoders to drive straight forward or backwards. This is a best effort,
		the results will not be perfect due to wheel slip and encoder errors.
			distance: inches, positive is forward, negative is backwards
			speed: Ranger's Encoder motor PWM speed, max value is 255
		"""
			
		speed = self.limit(speed, 0, 255)
		data = struct.unpack('4B', struct.pack('>2h', speed, distance))
		self.SCI.write(OPCODES.DRIVE_DIST_RANGER, data)


	def line_follow_Ranger(self):
		"""
		Line following mode of mBot Ranger
		Time of operating this mode is determined by main calling acript

		"""

		self.SCI.write(OPCODES.LINE_FOLLOW_RANGER)

	# ------------------------ LED ----------------------------

	def led(self, led_bits=0, power_color=0, power_intensity=0):
		"""
		led_bits: [check robot, dock, spot, debris]
		power_color: green [0] - red [255]
		power_instensity: off [0] - [255] full on

		All leds other than power are on/off.
		"""
		data = (led_bits, power_color, power_intensity)
		self.SCI.write(OPCODES.LED, data)

	def digit_led_ascii(self, display_string):
		"""
		This command controls the four 7 segment displays using ASCII character codes.

		Arguments:
			display_string: A four character string to be displayed. This must be four
				characters. Any blank characters should be represented with a space: ' '
				Due to the limited display, there is no control over upper or lowercase
				letters. create2api will automatically convert all chars to uppercase, but
				some letters (Such as 'B' and 'D') will still display as lowercase on the
				Create 2's display. C'est la vie. Any Create non-printable character
				will be replaced with a space ' '.
		"""
		#display_list = [' ']*4
		display_list = [32]*4
		for i, c in enumerate(display_string[:4]):
			val = ord(c.upper())
			if 32 <= val <= 126:
				display_list[i] = val
			else:
				# Char was not available. Just print a blank space
				display_list[i] = 32
				#display_list[i] = ' '

		#print('phrase:', ''.join(display_list))
		self.SCI.write(OPCODES.DIGIT_LED_ASCII, tuple(display_list))

	# ------------------------ Songs ----------------------------

	def createSong(self, song_num, notes):
		"""
		Creates a song

		Arguments
			song_num: 1-4
			notes: 16 notes and 16 durations each note should be held for (1 duration = 1/64 second)
		"""
		size = len(notes)
		if (2 > size > 32) or (size % 2 != 0):
			raise Exception('Songs must be between 1-16 notes and have a duration for each note')
		if 0 > song_num > 4:
			raise Exception('Song number must be between 0 and 4')

		if not isinstance(notes, tuple):
			notes = tuple(notes)

		# dur = 0.0
		# for i in notes:
		# 	if i % 2 != 0:
		# 		dur += notes[i]/64
		dt = 0
		for i in range(len(notes)//2):
			dt += notes[2*i+1]
		dt = dt/64

		msg = (song_num, size,) + notes
		# print('msg', msg)
		self.SCI.write(OPCODES.SONG, msg)

		return dt

	def playSong(self, song_num):
		"""
		Play a song
			Arguments
				song_num: 1-4
		"""
		if 0 > song_num > 4:
			raise Exception('Song number must be between 0 and 4')

		# print('let us play', song_num)

		self.SCI.write(OPCODES.PLAY, (song_num,))

	# ------------------------ Sensors ----------------------------

	def get_sensors(self):
		"""
		return: a namedtuple

		WARNING: now this only returns either pkt 0, 101 and 100, everything.
		"""

		opcode = OPCODES.SENSORS
		
		"""
		cmd = (100,)
		sensor_pkt_len = 80

		self.SCI.write(opcode, cmd)
		time.sleep(0.015)  # wait 15 msec
		packet_byte_data = self.SCI.read(sensor_pkt_len)

		sensors = SensorPacketDecoder(packet_byte_data)
		"""

		cmd = (0,)
		sensor_pkt_len = 26

		self.SCI.write(opcode, cmd)
		time.sleep(0.015)  # wait 15 msec
		packet_byte_data1 = self.SCI.read(sensor_pkt_len)

		sensors = SensorPacketDecoder_0(packet_byte_data1)

		"""
		cmd = (101,)
		sensor_pkt_len = 28

		self.SCI.write(opcode, cmd)
		time.sleep(0.015)  # wait 15 msec
		packet_byte_data2 = self.SCI.read(sensor_pkt_len)
		
		sensors = SensorPacketDecoder_101(packet_byte_data2)
		"""

		return sensors

	def getEncoderCountL(self):
		"""function to grab current encoder counts on left wheel
        	Parameters: None 
        	return: encoder count on left wheel (-32767 to 32767)
        	"""
		opcode = OPCODES.SENSORS
		cmd = (43,)
		sensor_pkt_len = 2

		self.SCI.write(opcode, cmd)
		time.sleep(0.015)  # wait 15 msec
		count_left = self.SCI.read(sensor_pkt_len)

		left_count = (ord(count_left[0])<<8) | ord(count_left[1])

		#print(left_count)

		return left_count

	def getEncoderCountR(self):
		"""function to grab current encoder counts on right wheel
        	Parameters: None
        	return: encoder count on right wheel (-32767 to 32767)
        	"""
		opcode = OPCODES.SENSORS
		cmd = (44,)
		sensor_pkt_len = 2

		self.SCI.write(opcode, cmd)
		time.sleep(0.015)  # wait 15 msec
		count_right = self.SCI.read(sensor_pkt_len)

		right_count = (ord(count_right[0])<<8) | ord(count_right[1])

		#print(right_count)

		return right_count

	def getEncoderCountLR(self):
		"""function to grab current encoder counts on both left and right wheel simultaneously
		Parameters: None 
		return: encoder count on left wheel (-32767 to 32767)
		"""
		opcode = OPCODES.QUERY_LIST
		cmd = (2,43,44,)
		sensor_pkt_len = 4

		self.SCI.write(opcode, cmd)
		time.sleep(0.015)  # wait 15 msec
		ret_bytes = self.SCI.read(sensor_pkt_len)

		left_count = (ord(ret_bytes[0])<<8) | ord(ret_bytes[1])
		right_count = (ord(ret_bytes[2])<<8) | ord(ret_bytes[3])

		#print(left_count, right_count)

		return left_count,right_count

	def getDistance(self):
		"""function to grab relative distance travelled in mm
        	Parameters: None
        	return: relative distance travelled in mm (-32767 to 32767)
			+ve value for forward direction and -ve for reverse direction
		"""
		opcode = OPCODES.SENSORS
		cmd = (19,)
		sensor_pkt_len = 2

		self.SCI.write(opcode, cmd)
		time.sleep(0.015)  # wait 15 msec
		dist_byte = self.SCI.read(sensor_pkt_len)

		distance = (ord(dist_byte[0])<<8) | ord(dist_byte[1])

		if distance > 32767:
			distance = 0 - (65535 - distance)
		#print(distance)

		return distance

	def getAngle(self):
		"""function to grab relative angle turned in degrees
        	Parameters: None
        	return: relative angle turned in degrees (-32767 to 32767)
			+ve value for CCW direction and -ve for CW direction
		"""
		opcode = OPCODES.SENSORS
		cmd = (20,)
		sensor_pkt_len = 2

		self.SCI.write(opcode, cmd)
		time.sleep(0.015)  # wait 15 msec
		angle_byte = self.SCI.read(sensor_pkt_len)

		angle = (ord(angle_byte[0])<<8) | ord(angle_byte[1])
 
		if angle > 32767:
			angle = 0 - (65535 - angle)

		#print(angle)

		return angle

