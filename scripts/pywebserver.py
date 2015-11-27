#!/usr/bin/env python
import SimpleHTTPServer
import SocketServer
import os
from rospkg import RosPack

rp = RosPack()
frontend_path = rp.get_path('rosimple')
frontend_path += '/frontend'

print "Changing serve path to: " + frontend_path

os.chdir(frontend_path)

PORT = 8123

Handler = SimpleHTTPServer.SimpleHTTPRequestHandler

httpd = SocketServer.TCPServer(("", PORT), Handler)

print "serving at port", PORT
httpd.serve_forever()