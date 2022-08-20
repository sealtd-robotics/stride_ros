#!/usr/bin/env python

from SocketServer import ThreadingMixIn
import rospy
import time
import BaseHTTPServer
import threading

PORT_NUMBER = 3001

class MyHandler(BaseHTTPServer.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.endswith('/test-data.csv'):
            self.send_response(200)
            self.send_header("Content-type", "text/csv")
            self.end_headers()
            
            with open("../../../test_log/data.csv", "rb") as f:
                self.wfile.write(f.read())

class ThreadedHTTPServer(ThreadingMixIn, BaseHTTPServer.HTTPServer):
    """handle requests in a separate thread"""

def create_server():
    httpd = ThreadedHTTPServer(("", PORT_NUMBER), MyHandler)
    print("Web server for data started at port {}".format(PORT_NUMBER))
    try:
        httpd.serve_forever()
    finally:
        httpd.server_close()
        print("Web server for data stopped at {}".format(PORT_NUMBER))

if __name__ == '__main__':
    node = rospy.init_node('web_server_for_data')

    # To allow ctrl+c to instantly close this ROS node, a thread is needed to host the server
    thread = threading.Thread(target=create_server)
    thread.setDaemon(True)
    thread.start()

    rospy.spin()