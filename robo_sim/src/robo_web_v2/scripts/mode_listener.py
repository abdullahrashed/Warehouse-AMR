#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

def mode_callback(data):
    rospy.loginfo("Received mode: %d", data.data)

def mode_listener():
    rospy.init_node('mode_listener', anonymous=True)
    rospy.Subscriber("mode_topic", Int16, mode_callback)
    rospy.spin()

if __name__ == '__main__':
    mode_listener()
