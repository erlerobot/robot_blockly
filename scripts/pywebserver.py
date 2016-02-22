#!/usr/bin/env python3
import http.server
import socketserver
import os
from rospkg import RosPack

rp = RosPack()
frontend_path = rp.get_path('robot_blockly')
frontend_path += '/frontend'

print("Changing serve path to: " + frontend_path)

os.chdir(frontend_path)

PORT = 80

Handler = http.server.SimpleHTTPRequestHandler

httpd = socketserver.TCPServer(("", PORT), Handler)

print("serving at port", PORT)
httpd.serve_forever()
