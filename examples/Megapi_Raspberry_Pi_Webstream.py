#!/home/pi/.virtualenvs/py3cv4/bin/python

from __future__ import print_function
import pycreate2
import socketserver, threading, time
import struct

import io
import picamera
import logging
import datetime as dt
import os
from threading import Condition
from http import server

# Web Server Streaming Section #
PAGE="""\
<html>
<head>
<title>Raspberry Pi - Surveillance Camera</title>
</head>
<body>
<center><h1>Raspberry Pi - Surveillance Camera</h1><center>
<cleft><img src="stream.mjpg" width="640" height="480" /><left>
<right><img src="/Pictures/videostill_1.jpg" width="640" height="480" /><right>
</body>
</html>
"""

keep_running = True
Obstacle_detect = False

class StreamingOutput(object):
    def __init__(self):
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = Condition()

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame, copy the existing buffer's content and notify all
            # clients it's available
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)

class ThreadedStreamingHandler(server.BaseHTTPRequestHandler):

    def do_GET(self):

        global keep_running
        global Obstacle_detect

        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/Pictures/videostill_1.jpg':
            path_to_image = os.getcwd() + self.path
            if os.path.isfile(path_to_image):
                img = open(path_to_image, 'rb')
                statinfo = os.stat(path_to_image)
                img_size = statinfo.st_size
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', img_size)
                self.end_headers()
                self.wfile.write(img.read())
                self.wfile.write(b'\r\n')
                img.close()              
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while keep_running == True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
                    
                    if Obstacle_detect == True:

                        timestamp = dt.datetime.now().strftime('%m-%d-%Y-%H:%M:%S')
                        videostillfilename='./Pictures/videostill_' + timestamp + '.jpg'
                        with open(videostillfilename, "wb") as outfile:
                            outfile.write(frame)
    
                        videostillfilename='./Pictures/videostill_1' + '.jpg'
                        with open(videostillfilename, "wb") as outfile:
                            outfile.write(frame)

                        Obstacle_detect = False

                    if keep_running == False:

                        Obstacle_detect = True
                        keep_running = True
                    
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()

class ThreadedStreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

# UDP Server Section #
bot = None
udp_client_address = None
udp_socket = None

class ThreadedUDPRequestHandler(socketserver.BaseRequestHandler):

    def handle(self):

        global bot
        global udp_client_address
        global udp_socket
        global keep_running
        
        data = self.request[0].strip()
        udp_client_address = self.client_address
        udp_socket = self.request[1]
        current_thread = threading.current_thread()
        
        #print("{}: client: {}, wrote: {}".format(current_thread.name, self.client_address, data))
        #udp_socket.sendto(data.upper(), self.client_address)

        print(data)
        bot.SCI.ser.write(data)

class ThreadedUDPServer(socketserver.ThreadingMixIn, socketserver.UDPServer):
    pass

if __name__ == "__main__":

    # Initialise Pi camera and start up Web Server for streaming video
    camera = picamera.PiCamera(resolution='640x480', framerate=24)
    output = StreamingOutput()
    camera.rotation = 180
    camera.start_recording(output, format='mjpeg')

    timestamp = dt.datetime.now().strftime('%m-%d-%Y-%H:%M:%S')
    videostillfilename='./Pictures/videostill_' + timestamp + '.jpg'
    camera.capture(videostillfilename, use_video_port=True)
    print('Captured image: ' + videostillfilename)

    videostillfilename='./Pictures/videostill_1' + '.jpg'
    camera.capture(videostillfilename, use_video_port=True)

    camera.wait_recording(2.0)
   
    web_server_address = ('', 8000)
    web_server = ThreadedStreamingServer(web_server_address, ThreadedStreamingHandler)
    web_server_thread = threading.Thread(target=web_server.serve_forever)
    web_server_thread.daemon = True
    
    # Create2 or Ranger Bot Setup
    config = {}

    config["transport"] = ''       # For use with serial port interface
    config["robot"] = 'Ranger'
    config["logfile_name"] = 'Megapi_Ranger_U1_log_'

    config["port"] = '/dev/ttyAMA0'  # this is the serial port on Raspberry Pi 3
    config["baud"] = 115200
        
    bot = pycreate2.Create2(config)
    bot.SCI.buffersize = 1024 # Currently, this parameter is only defined for UDP interface

    bot.start()
    bot.safe()

    # UDP Server Setup
    HOST, PORT = "0.0.0.0", 1025

    server = ThreadedUDPServer((HOST, PORT), ThreadedUDPRequestHandler)

    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.daemon = True

    try:
        web_server_thread.start()
        print("Web Server started at port 8000")

        server_thread.start()
        print("Server started at {} port {}".format(HOST, PORT))

        while True:
            #time.sleep(5.0)
            #print("Going to check the serial port for messages")

            #data_read = bot.SCI.read(bot.SCI.buffersize).decode("utf-8",errors="ignore")
            data_read = bot.SCI.read(bot.SCI.buffersize)
            data_read_string = data_read.decode("utf-8",errors="ignore")
            if data_read_string is not "":
                print(data_read_string)
                if "Obstacle" in data_read_string:
                    #time.sleep(0.01)
                    keep_running = False

                if udp_client_address is not None:
                    udp_socket.sendto(data_read, udp_client_address)
            
    except (KeyboardInterrupt, SystemExit):
        server.shutdown()
        server.server_close()
        bot.drive_stop()        
        time.sleep(0.5)
        camera.stop_recording()
        web_server.shutdown()
        web_server.server_close()

    print('Shutting down ... bye')