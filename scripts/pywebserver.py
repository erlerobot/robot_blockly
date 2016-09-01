#!/usr/bin/env python
import SimpleHTTPServer
import SocketServer
import socket
import os
from rospkg import RosPack
import rospy
import json

rospy.init_node("robot_blockly_webserver")

rp = RosPack()
frontend_path = rp.get_path('robot_blockly')
frontend_path += '/frontend'

rospy.loginfo("Changing server path to: " + frontend_path)

os.chdir(frontend_path)

block_packages_list = rospy.get_param('~block_packages')
block_packages_paths = ["/" + package for package in block_packages_list]
block_javascript_files = [package + "/blocks_uncompressed.js" for package in block_packages_paths]
if block_javascript_files is None:
    block_javascript_files = "EMPTY"

HOST = socket.gethostname()
PORT = 8000
address = ("",PORT)


class MyRequestHandler(SimpleHTTPServer.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith(tuple(block_packages_paths)):
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
        data = json.loads(data_string)

        if data is not None and all(name in data for name in ['module', 'method', 'parameters']):
            module = data["module"]
            method = data["method"]
            parameters = data["parameters"]

            if "robot_blockly" == module:
                if "get_block_packages" == method:
                    result = block_javascript_files
                else:
                    result = {'error': 'Unknown method name'}
            elif module in block_packages_list:
                try:
                    called_module = __import__(module + ".web_service_module", fromlist=['WebServiceModule'])
                    try:
                        web_handler_class = getattr(called_module, 'WebServiceModule')
                        instance = web_handler_class()
                        try:
                            function_to_call = getattr(instance, method)
                            result = function_to_call(parameters)
                        except AttributeError:
                            result = {'error': 'class WebServiceModule does not have called function'}
                    except AttributeError:
                        result = {'error': 'Module does not have class WebServiceModule'}
                except ImportError:
                    result = {'error': 'Module does not have web service methods'}
            else:
                result = {'error': 'Incorrect module name'}
        else:
            result = {'error': 'Incorrect web service request body'}

        self.wfile.write(json.dumps(result))


Handler = MyRequestHandler
httpd = SocketServer.TCPServer(address, Handler)

print("serving at port", PORT)
httpd.serve_forever()
