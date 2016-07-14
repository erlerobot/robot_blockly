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


class MyRequestHandler(SimpleHTTPServer.SimpleHTTPRequestHandler):
    def do_GET(self):
        block_path = rp.get_path('sr_blockly_blocks')
        if self.path.startswith('/sr_blockly_blocks'):
            block_path = os.path.split(block_path)[0]
            os.chdir(block_path)
        else:
            os.chdir(frontend_path)
        print self.path
        return SimpleHTTPServer.SimpleHTTPRequestHandler.do_GET(self)

Handler = MyRequestHandler
httpd = SocketServer.TCPServer(address, Handler)

print("serving at port", PORT)
httpd.serve_forever()
