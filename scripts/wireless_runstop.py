#!/usr/bin/env python3

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
            rospy.loginfo(f"Trying to connect to e-stop at {host}:{port}...")
            s = socket.socket()
            s.settimeout(timeout)
            s.connect((host, port))

            # return socket
            rospy.loginfo(f"Connected to e-stop!")
            return s

        except (socket.timeout, socket.error) as e:
            rospy.logwarn(f"Connection failed: {e}. Retrying after {timeout} seconds...")
            rospy.sleep(timeout)

if __name__ == '__main__':
    pub = rospy.Publisher('enable_software_runstop', Bool, queue_size=1)
    rospy.init_node('wireless_estop')

    rospy.loginfo(f"Engaging runstop until connected to wireless e-stop")
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
                rospy.logwarn(f"e-stop engaged")
                pub.publish(True)
            else:
                pub.publish(False)

        except (socket.timeout, socket.error) as e:
            rospy.logwarn(f"Connection lost: {e}. Engaging failsafe and reconnecting...")
            pub.publish(True)
            s.close()
            s = None
