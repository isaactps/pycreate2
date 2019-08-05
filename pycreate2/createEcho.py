# The MIT License
#
# Copyright (c) 2017 Kevin Walchko
#
# This is basically the interface between the Create2 and pyserial

from __future__ import print_function
from __future__ import division
from pycreate2.OI import OPCODES
from pycreate2.OI import OPCODES_STRINGS
from pycreate2.OI import DRIVE
import struct
import io
import pycreate2.fifo 

class EchoCommandInterface(object):
	"""
	This class handles sending commands to the Create2. Writes will take in tuples
	and format the data to transfer to the Create.
	"""

	def __init__(self):
		"""
		Constructor.

		Creates the serial port, but doesn't open it yet. Call open() to open
		it.
		"""
		#self.ser = serial.Serial()
		#self.readf = io.BytesIO('XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX')

		self.is_open = False

	def __del__(self):
		"""
		Destructor.

		Closes the serial port
		"""
		self.close()

	def open(self, timeout=1):
		"""
		Opens a serial port to the create.

		port: the serial port to open, ie, '/dev/ttyUSB0'
		baud: default is 115200, but can be changed to a lower rate via the create api
		"""
		if self.is_open:
			self.close()

		print('-'*40)
		print('Connected')
		print('-'*40)

		self.is_open = True

	def write(self, opcode, data=None):
		"""
		Writes a command to the create. There needs to be an opcode and optionally
		data. Not all commands have data associated with it.

		opcode: see creaet api
		data: a tuple with data associated with a given opcode (see api)
		"""
		msg = (opcode,)

		# Sometimes opcodes don't need data. Since we can't add
		# a None type to a tuple, we have to make this check.
		if data:
			msg += data

		#print(msg)

		#print([hex(ord(c)) for c in data])
		pkt = struct.pack('B' * len(msg), *msg)
		cmd = OPCODES_STRINGS[opcode]
		if opcode == OPCODES.DIGIT_LED_ASCII:
			print(cmd, ''.join(chr(ord(c)) for c in pkt[1:]))
		elif opcode == OPCODES.DRIVE:
			vel = ord(pkt[1])<<8 | ord(pkt[2])
			rad = ord(pkt[3])<<8 | ord(pkt[4])
			if vel > 0x8000: 
				vel = vel-(1<<16)
			if rad > 0x8000: 
				rad = rad-(1<<16)

			#STRAIGHT=0x8000, STRAIGHT_ALT=0x7FFF, TURN_CW=-1, TURN_CCW=0x0001)
			if rad == DRIVE.STRAIGHT:
				rad_string = "STRAIGHT"
			elif rad == DRIVE.STRAIGHT_ALT:
				rad_string = "STRAIGHT_ALT"
			elif rad == DRIVE.TURN_CW:
				rad_string = "TURN_CW"
			elif rad == DRIVE.TURN_CCW:
				rad_string = "TURN_CCW"
			else:
				rad_string = str(rad)
			print(cmd, vel, rad_string)
		elif opcode == OPCODES.DRIVE_PWM:
			rt = ord(pkt[1])<<8 | ord(pkt[2])
			lt = ord(pkt[3])<<8 | ord(pkt[4])
			if rt > 0xFF: 
				rt = rt-(1<<8)
			if lt > 0xFF: 
				lt = lt-(1<<8)
			print(cmd, rt, lt)
		elif opcode == OPCODES.LED:
			bits = hex(ord(pkt[1])&0xF)
			color = ord(pkt[2])
			intensity = ord(pkt[3])
			print(cmd, bits, color, intensity)
		elif opcode == OPCODES.SENSORS:
			pkt_id = ord(pkt[1])
			print(cmd, pkt_id)
		else:
			print(cmd)
                    
		#print(pkt)

	def read(self, num_bytes):
		"""
		Read a string of 'num_bytes' bytes from the robot.

		Arguments:
			num_bytes: The number of bytes we expect to read.
		"""
		if not self.is_open:
			raise Exception('You must open the first')

		#data = self.ser.read(num_bytes)
		print("Reading ",num_bytes)
		#data = self.readf.read(num_bytes)
		data = '0'*num_bytes
		print("data ",len(data))

		return data

	def close(self):
		"""
		Closes the serial connection.
		"""
		if self.is_open:
			print('Closing')
