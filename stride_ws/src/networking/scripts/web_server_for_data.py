#!/usr/bin/env python

from http.server import HTTPServer
from socketserver import ThreadingMixIn
import rospy
import time
import BaseHTTPServer
import threading


HOST_NAME = ""
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
    # server_class = BaseHTTPServer.HTTPServer
    # httpd = server_class((HOST_NAME, PORT_NUMBER), MyHandler)
    httpd = ThreadedHTTPServer((HOST_NAME, PORT_NUMBER), MyHandler)
    print("Web server for data started at port {}".format(PORT_NUMBER))
    try:
        httpd.serve_forever()
    finally:
        httpd.server_close()
        print("Web server for data stopped at {}".format(PORT_NUMBER))

if __name__ == '__main__':
    node = rospy.init_node('web_server_for_data')
    thread = threading.Thread(target=create_server)
    thread.setDaemon(True)
    thread.start()

    rospy.spin()

# if __name__ == '__main__':
#     node = rospy.init_node('web_server_for_data')

#     server_class = BaseHTTPServer.HTTPServer
#     httpd = server_class((HOST_NAME, PORT_NUMBER), MyHandler)
#     print("Web server for data started at port {}".format(PORT_NUMBER))
#     try:
#         httpd.serve_forever()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         print("Web server for data stopped at {}".format(PORT_NUMBER))
