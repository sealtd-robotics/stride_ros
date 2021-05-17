#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def log(msg):
    print msg

def unregister_pub1(event=None):
    global pub1_flag
    pub1_flag = False
    print 'Unregister Pub 1 ...'
    pub1.unregister()

def unregister_pub2(event=None):
    global pub2_flag
    pub2_flag = False
    print 'Unregister Pub 2 ...'
    pub2.unregister()

def recreate_pub2(event=None):
    global pub2
    print 'Re-create pub2'
    pub2 = rospy.Publisher('topic2', String, queue_size=2)
    pub2_flag = True

def finish(event=None):
    global not_finished
    not_finished = False


rospy.init_node('pub_bug')

# A publisher to test unregister with no subscriber
pub1 = rospy.Publisher('topic1', String, queue_size=1)
pub1_flag = True

# Two publishers to the same topic with one subscriber
pub2 = rospy.Publisher('topic2', String, queue_size=2)
# pub3 = rospy.Publisher('topic2', String, queue_size=2)
sub  = rospy.Subscriber('topic2', String, callback=log)
pub2_flag = True

# Unregister pub1 after 5 seconds
rospy.Timer(rospy.Duration(5), unregister_pub1, oneshot=True)

# Unregister pub2 after 10 seconds
rospy.Timer(rospy.Duration(10), unregister_pub2, oneshot=True)

# Create a new publisher to topic2 after 15 seconds
rospy.Timer(rospy.Duration(15), recreate_pub2, oneshot=True)

# Stop everything after 20 seconds
not_finished = True
rospy.Timer(rospy.Duration(20), finish, oneshot=True)

# Publish periodically
while not rospy.is_shutdown() and not_finished:
    if pub1_flag:
        pub1.publish('hello from pub1')
    if pub2_flag:
        pub2.publish('hello from pub2')
    # pub3.publish('hello from pub3')
    rospy.rostime.wallsleep(0.5)