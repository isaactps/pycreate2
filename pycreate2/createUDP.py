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

class UDPCommandInterface(object):
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
		self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.buffersize = 1024
		self.message = 'Ping UDP'
		self.socket_timeout = 25
		self.udp.settimeout(self.socket_timeout)

		self.start = time.asctime(time.localtime(time.time()))

	def __del__(self):
		"""
		Destructor.

		Closes the UDP port
		"""
		self.close()

	def open(self, server_ip_addr, logfile_name, timeout=1):
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
			self.remote_ip = '192.168.1.24'

		self.portnumber = 1025
		self.udp.sendto(self.message.encode(), (self.remote_ip , self.portnumber))

		self.logfile_name = logfile_name + self.start
		with open(self.logfile_name, 'w') as f:
    			print('Opening log file at time {}'.format(self.start), file=f)

		self.is_open = True

		#Wait for first Link Alive message from Robot
		data_ack = self.read(self.buffersize)

		self.socket_timeout = 20 #Reduce the timeout period
		
		print('-'*40)
		print('Connected')
		print('-'*40)

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
		print(data)

		self.udp.sendto(data,(self.remote_ip , self.portnumber))
		#print("Sent data")

		#Wait for Ack Message from Robot
		data_ack = self.read(self.buffersize)

	def read(self, num_bytes):
		"""
		Read a string of 'num_bytes' bytes from the robot.

		Arguments:
			num_bytes: The number of bytes we expect to read.
		"""
		if not self.is_open:
			raise Exception('You must first open the UDP port')

		#print("Start reading data")
		
		try:
			data, server = self.udp.recvfrom(num_bytes)
			end = time.time()

			rx_msg = "Msg is {}".format(data)
			print(rx_msg + " " + "at local time " + time.asctime(time.localtime(end)))
			with open(self.logfile_name, 'a') as f:
				print(rx_msg + " " + "at local time " + time.asctime(time.localtime(end)), file=f)

			return data

			#If data is not received back from server, print Timed out message
		except socket.timeout:
			print('REQUEST TIMED OUT')
			self.udp.sendto(self.message.encode(), (self.remote_ip , self.portnumber))

			return -1

	def close(self):
		"""
		Closes the serial connection.
		"""
		if self.is_open:
			print('Closing UDP port')
			self.udp.close()
