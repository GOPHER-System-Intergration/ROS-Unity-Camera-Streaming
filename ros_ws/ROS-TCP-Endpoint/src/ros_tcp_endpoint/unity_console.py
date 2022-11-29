#!/usr/bin/env python3
# Basics ROS program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
import numpy as np

from ROS_TCP_Endpoint_msgs.msg import ConsoleInfo
from ROS_TCP_Endpoint_msgs.srv import UnityPrint,UnityPrintResponse


def handle_print(req):
    msg = ConsoleInfo()
    msg.header.stamp = rospy.Time.now()
    msg.content = req.content
    msg.code = req.code
    pub.publish(msg)
    return UnityPrintResponse(0)

def unity_console_server():
    rospy.init_node('unity_console_server')
    s = rospy.Service('unity_print', UnityPrint, handle_print)
    global pub
    pub = rospy.Publisher('unity_console', ConsoleInfo, queue_size=10)
    print("Spining up console server")
    rospy.spin()

if __name__ == "__main__":
    unity_console_server()

    
 
