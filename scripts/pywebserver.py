#!/usr/bin/env python
import SimpleHTTPServer
import SocketServer
import socket
import os
from rospkg import RosPack
import rospy

rospy.init_node("robot_blockly_webserver")

rp = RosPack()
frontend_path = rp.get_path('robot_blockly')
frontend_path += '/frontend'

rospy.loginfo("Changing server path to: " + frontend_path)

os.chdir(frontend_path)

block_packages = rospy.get_param('~block_packages')
block_packages = ["/" + package for package in block_packages]
block_javascript_files = ";".join([package + "/blocks_uncompressed.js" for package in block_packages])
if "" == block_javascript_files:
    block_javascript_files = "EMPTY"

HOST = socket.gethostname()
PORT = 8000
address = ("",PORT)


class MyRequestHandler(SimpleHTTPServer.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith(tuple(block_packages)):
            t = next(part for part in self.path.split(os.path.sep) if part)
            block_path = rp.get_path(t)
            block_path = os.path.split(block_path)[0]
            os.chdir(block_path)
        else:
            os.chdir(frontend_path)
        return SimpleHTTPServer.SimpleHTTPRequestHandler.do_GET(self)

    def do_POST(self):
        length = int(self.headers.getheader('content-length'))
        data_string = self.rfile.read(length)
        if "get_block_packages" == data_string:
            result = block_javascript_files
        else:
            result = "Usupported method"
        self.wfile.write(result)


Handler = MyRequestHandler
httpd = SocketServer.TCPServer(address, Handler)

print("serving at port", PORT)
httpd.serve_forever()
