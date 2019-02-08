# The MIT License
#
# Copyright (c) 2017 Kevin Walchko
#
# This is basically the interface between the Create2 and pyserial

from __future__ import print_function
from __future__ import division
import struct
import io
import paho.mqtt.client as mqtt
import fifo 

class MQTTCommandInterface(object):
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
		#self.ser = serial.Serial()
                #self.readf = io.BytesIO('XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX')

                self.is_open = False
		self.mqttc = mqtt.Client()
                self.readf = fifo.BytesFIFO(100)
		self.mqttc.on_message = self.on_message
		self.mqttc.on_connect = self.on_connect
		self.mqttc.on_publish = self.on_publish
		self.mqttc.on_subscribe = self.on_subscribe


	def on_connect(self, mqttc, obj, flags, rc):
	    print("on_connect rc: " + str(rc))


	def on_message(self, mqttc, obj, msg):
	    print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))
            print(self.readf.write(msg.payload))

	def on_publish(self, mqttc, obj, mid):
	    print("mid: " + str(mid))


	def on_subscribe(self, mqttc, obj, mid, granted_qos):
	    print("Subscribed: " + str(mid) + " " + str(granted_qos))


	def on_log(self, mqttc, obj, level, string):
	    print(string)

	def __del__(self):
		"""
		Destructor.

		Closes the serial port
		"""
		self.close()

	def open(self, port, timeout=1):
		"""
		Opens a serial port to the create.

		port: the serial port to open, ie, '/dev/ttyUSB0'
		buad: default is 115200, but can be changed to a lower rate via the create api
		"""
		#self.ser.port = port
		#self.ser.baudrate = baud
		#self.ser.timeout = timeout
		# print self.ser.name
		if self.is_open:
                        self.close()

		self.mqttc.connect("localhost", 1883)
		self.mqttc.subscribe("topic2", 0)
                self.mqttc.loop_start()

		#self.mqttc.loop_forever()

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

                print(msg)

		data = struct.pack('B' * len(msg), *msg)
		print([hex(ord(c)) for c in data])
                self.mqttc.publish("topic1", data)
		#self.write(data)
		#self.write(struct.pack('B' * len(msg), *msg))

	def read(self, num_bytes):
		"""
		Read a string of 'num_bytes' bytes from the robot.

		Arguments:
			num_bytes: The number of bytes we expect to read.
		"""
		if not self.is_open:
			raise Exception('You must open the mqtt topic first')

		#data = self.ser.read(num_bytes)
                print("Reading ",num_bytes)
                data = self.readf.read(num_bytes)
                print("data ",len(data))

		return data

	def close(self):
		"""
		Closes the serial connection.
		"""
		if self.is_open:
                    print('Closing MQTT')
                    self.mqttc.disconnect()
