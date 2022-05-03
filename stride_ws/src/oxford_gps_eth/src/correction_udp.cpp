#include <iostream>
#include "dispatch.h"
#include <ros/ros.h>
#include "rs232/rs232.h"
#include <arpa/inet.h>

static inline int readSocket(int fd, unsigned int timeout, void *data, size_t size, sockaddr *source_ptr = NULL)
{
  if (fd >= 0) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    // Set up timeout
    struct timeval tv;
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = (timeout * 1000) % 1000000;

    if (select(fd + 1, &fds, NULL, NULL, &tv) > 0) {
      socklen_t socklen = source_ptr ? sizeof(*source_ptr) : 0;
      socklen_t *socklen_ptr = source_ptr ? &socklen : NULL;
      return recvfrom(fd, data, size, 0, source_ptr, socklen_ptr);
    }

    // Timeout
    return 0;
  }
  return -1;
}


void spin_correction(int fd, unsigned int timeout, void *data, size_t size, sockaddr *source_ptr = NULL) {
    int byte_received = 0;
    while (ros::ok()) {
        byte_received = readSocket(fd, timeout, data, size, source_ptr);
        // if ( (byte_received = readSocket(fd, timeout, data, size, source_ptr) ) > 0) {
        if ( byte_received) {
            SendBuf((unsigned char*)data, byte_received);
            // printf("%d\n", byte_received);
        } 
        // else {
        //     // printf("Nothing");
        // }
    }
}