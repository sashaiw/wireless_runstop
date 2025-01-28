#!/usr/bin/env python2

import rospy
from std_msgs.msg import Bool

import socket
import time

def connect_to_estop():
    while not rospy.is_shutdown():
        # get host and port
        host = rospy.get_param("host", "fetch-runstop")
        port = rospy.get_param("port", 1337)
        timeout = rospy.get_param("timeout", 2)

        try:
            # attempt connection to estop
            rospy.loginfo("Trying to connect to e-stop at {0}:{1}...".format(host, port))
            s = socket.socket()
            s.settimeout(timeout)
            s.connect((host, port))

            # return socket
            rospy.loginfo("Connected to e-stop!")
            return s

        except (socket.timeout, socket.error) as e:
            rospy.logwarn("Connection failed: {0}. Retrying after {1} seconds...".format(e, timeout))
            rospy.sleep(timeout)

if __name__ == '__main__':
    pub = rospy.Publisher('enable_software_runstop', Bool, queue_size=1)
    rospy.init_node('wireless_estop')

    rospy.loginfo("Engaging runstop until connected to wireless e-stop")
    pub.publish(True)

    s = connect_to_estop()

    while not rospy.is_shutdown():
        if s is None:
            rospy.logwarn("Reconnecting...")
            pub.publish(True)
            s = connect_to_estop()
            continue

        try:
            estop_pressed = s.recv(1024) == b'\x00'

            if estop_pressed:
                rospy.logwarn("e-stop engaged")
                pub.publish(True)
            else:
                pub.publish(False)

        except (socket.timeout, socket.error) as e:
            rospy.logwarn("Connection lost: {0}. Engaging failsafe and reconnecting...".format(e))
            pub.publish(True)
            s.close()
            s = None
