#!/usr/bin/env python

# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage

import string,cgi,time
from os import curdir, sep
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from SocketServer import ThreadingMixIn
import cv
import re
from threading import Thread, Lock

VERBOSE=True

import copy

mutex = Lock()
topic_name = "/usb_cam/image_raw/compressed"
class web_video_server:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # subscribed Topic
        self.subscriber = rospy.Subscriber(topic_name,
            CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            print "subscribed to " + topic_name

    def getCompressedImage(self):
        mutex.acquire(1)
        result = copy.deepcopy(self.np_arr);
        mutex.release()
        return result;

    def callback(self, ros_data):
        '''Callback function of subscribed topic. '''
        mutex.acquire(1)
        self.np_arr = ros_data.data;
        mutex.release()

class MyHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        try:
            self.path=re.sub('[^.a-zA-Z0-9]', "",str(self.path))
            if self.path=="" or self.path==None or self.path[:1]==".":
                return
            if self.path.endswith(".html"):
                f = open(curdir + sep + self.path)
                self.send_response(200)
                self.send_header('Content-type',    'text/html')
                self.end_headers()
                self.wfile.write(f.read())
                f.close()
                return
            if self.path.endswith(".mjpeg"):
                self.send_response(200)
                self.wfile.write("Content-Type: multipart/x-mixed-replace; boundary=--aaboundary")
                self.wfile.write("\r\n\r\n")
                while 1:
                  JpegData=self.ic.getCompressedImage();                    
                  self.wfile.write("--aaboundary\r\n")
                  self.wfile.write("Content-Type: image/jpeg\r\n")
                  self.wfile.write("Content-length: "+str(len(JpegData))+"\r\n\r\n" )
                  self.wfile.write(JpegData)
                  self.wfile.write("\r\n\r\n\r\n")    
                  time.sleep(0.05)
                return
            if self.path.endswith("*.jpeg"):
                self.send_response(200)
                self.wfile.write("Content-Type: multipart/x-mixed-replace; boundary=--aaboundary")
                self.wfile.write("\r\n\r\n")
                JpegData=self.ic.getCompressedImage();
                self.wfile.write("--aaboundary\r\n")
                self.wfile.write("Content-Type: image/jpeg\r\n")
                self.wfile.write("Content-length: "+str(len(JpegData))+"\r\n\r\n" )
                self.wfile.write(JpegData)
                self.wfile.write("\r\n\r\n\r\n")
                return
            return
        except IOError:
            self.send_error(404,'File Not Found: %s' % self.path)
        except :
            pass

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""

def main_server(args, ic):
    try:
        MyHandler.ic = ic
        server = ThreadedHTTPServer(('0.0.0.0', 8080), MyHandler)
        print 'started httpserver...'
        server.serve_forever()
    except KeyboardInterrupt:
        print '^C received, shutting down server'
        server.socket.close()  


def main_ros(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('web_video_server', anonymous=False)
    print 'started ROS...'
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    ic = web_video_server()

    t = Thread(target=main_server, args=(sys.argv, ic))
    t.start()
    main_ros(sys.argv)