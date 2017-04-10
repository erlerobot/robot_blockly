#!/usr/bin/env python3
import http.server
import socketserver
import socket
import os
from rospkg import RosPack

rp = RosPack()
frontend_path = rp.get_path('robot_blockly')
frontend_path += '/frontend'

print("Changing serve path to: " + frontend_path)

os.chdir(frontend_path)

HOST = socket.gethostname()
PORT = 1036
address = ("",PORT)

Handler = http.server.SimpleHTTPRequestHandler

httpd = socketserver.TCPServer(address, Handler)

print("serving at port", PORT)
httpd.serve_forever()
