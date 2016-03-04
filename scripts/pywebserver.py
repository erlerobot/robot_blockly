#!/usr/bin/env python
import SimpleHTTPServer
import SocketServer
import socket
import os
from rospkg import RosPack

rp = RosPack()
frontend_path = rp.get_path('robot_blockly')
frontend_path += '/frontend'

print("Changing serve path to: " + frontend_path)

os.chdir(frontend_path)

HOST = socket.gethostname()
PORT = 8000
address = ("",PORT)

httpd = SocketServer.TCPServer(address, SimpleHTTPServer.SimpleHTTPRequestHandler)

print("serving at port", PORT)
httpd.serve_forever()
