#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

def echo(data):
    print(data)

rospy.init_node('PID', anonymous=True)
rospy.Subscriber('fine_path', Float32, echo)
rospy.spin()