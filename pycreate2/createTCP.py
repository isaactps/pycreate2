# The MIT License
#
# Copyright (c) 2017 Kevin Walchko
#
# This is basically the interface between the Create2 and pyserial

from __future__ import print_function
from __future__ import division
import struct
import io
import socket
import string
import time 
import sys

from pycreate2.fifo import BytesFIFO 

class TCPCommandInterface(object):
	"""
	This class handles sending commands to the Create2. Writes will take in tuples
	and format the data to transfer to the Create.
	"""

	def __init__(self):
		"""
		Constructor.

		Creates the serial port, but doesn't open it yet. Call open(port) to open
		it.
		"""
		self.is_open = False
		self.tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.readf = fifo.BytesFIFO(100)


	def __del__(self):
		"""
		Destructor.

		Closes the serial port
		"""
		self.close()

	def open(self, server_ip_addr, timeout=1):
		"""
		Opens a serial port to the create.

		port: the serial port to open, ie, '/dev/ttyUSB0'
		baud: default is 115200, but can be changed to a lower rate via the create api
		"""
		if self.is_open:
                	self.close()

		if server_ip_addr != "":
			self.remote_ip = server_ip_addr
		else:		
			self.remote_ip = '192.168.1.97' #'10.0.0.1'

		self.portnumber = 9001
		self.tcp.connect((self.remote_ip , self.portnumber))
		
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
		length_msg = len(msg)

		data = struct.pack('B' * length_msg, *msg)
		#print([hex(ord(c)) for c in data])

		self.tcp.sendall(data)
		#print("Sent data")

		"""sent = 0
		while sent < length_msg:
			#print(hex(ord(data[sent])))			
			self.tcp.send(data[sent])
			sent += 1"""

	def read(self, num_bytes):
		"""
		Read a string of 'num_bytes' bytes from the robot.

		Arguments:
			num_bytes: The number of bytes we expect to read.
		"""
		if not self.is_open:
			raise Exception('You must first open the TCP port')

		print("Start reading data")
		rem_bytes = num_bytes
		while (rem_bytes > 0):
			data = self.tcp.recv(rem_bytes)
			time.sleep(0.025)
			self.readf.write(data)
			rem_bytes -= len(data)

		data = self.readf.read(num_bytes)

		#print("Reading ",num_bytes)
		#print("Data length ",len(data))

		return data

	def close(self):
		"""
		Closes the serial connection.
		"""
		if self.is_open:
			print('Closing TCP port')
			self.tcp.shutdown(socket.SHUT_RDWR)                    
			self.tcp.close()
