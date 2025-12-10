#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import String

def callback(msg):
    try:
        data = json.loads(msg.data)
        temp = data.get('temperature')
        humidity = data.get('humidity')
        rospy.loginfo(f"🌡️ Temp: {temp}°C  💧 Humidity: {humidity}%")
    except Exception as e:
        rospy.logerr(f"Failed to parse environment data: {e}")

def listener():
    rospy.init_node('env_printer', anonymous=True)
    rospy.Subscriber('/sensor_data', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
