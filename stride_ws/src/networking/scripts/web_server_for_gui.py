#!/usr/bin/env python

# ========================================================================
# Copyright (c) 2022, SEA Ltd.
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree. 
# ========================================================================

from http.server import SimpleHTTPRequestHandler
from socketserver import ThreadingMixIn, TCPServer
import rospy
import BaseHTTPServer
import threading
import os

PORT_NUMBER = 3000
gui_directory = '../../../../stride_gui_build/build'

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
    node = rospy.init_node('web_server_for_gui')

    os.chdir(gui_directory)

    # To allow ctrl+c to instantly close this ROS node, a thread is needed to host the server
    thread = threading.Thread(target=create_server)
    thread.setDaemon(True)
    thread.start()

    rospy.spin()