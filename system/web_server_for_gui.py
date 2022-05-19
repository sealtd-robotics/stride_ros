#!/usr/bin/env python

from http.server import SimpleHTTPRequestHandler
from socketserver import ThreadingMixIn, TCPServer
import BaseHTTPServer
import threading
import os

PORT_NUMBER = 3000
gui_directory = '/home/nvidia/stride_ros/stride_gui_build/build'

Handler = SimpleHTTPRequestHandler

class ThreadedHTTPServer(ThreadingMixIn, TCPServer):
    """handle requests in a separate thread"""

def create_server():
    httpd = ThreadedHTTPServer(("", PORT_NUMBER), Handler)
    print("Web server for GUI started at port {}".format(PORT_NUMBER))
    try:
        httpd.serve_forever()
    finally:
        httpd.server_close()
        print("Web server for GUI stopped at {}".format(PORT_NUMBER))

if __name__ == '__main__':
    os.chdir(gui_directory)
    create_server()
    print("Server stopped")