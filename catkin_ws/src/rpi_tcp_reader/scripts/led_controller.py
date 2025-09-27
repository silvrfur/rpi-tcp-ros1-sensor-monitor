#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import json

def callback(msg):
    data = json.loads(msg.data)
    if data['is_dark']:
        rospy.loginfo("It is dark! Turn ON LED or take action")
    else:
        rospy.loginfo("It is bright! Turn OFF LED or take action")

rospy.init_node('led_controller')
rospy.Subscriber('sensor_data', String, callback)
rospy.spin()
