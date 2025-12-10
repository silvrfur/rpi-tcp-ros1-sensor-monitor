#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import String

def callback(msg):
    try:
        data = json.loads(msg.data)
        gas = data.get('gas_detected', False)
        if gas:
            rospy.logwarn("⚠️ GAS DETECTED! Take immediate action!")
        else:
            rospy.loginfo("✅ Air quality normal.")
    except Exception as e:
        rospy.logerr(f"Failed to parse gas data: {e}")

def listener():
    rospy.init_node('gas_detector', anonymous=True)
    rospy.Subscriber('/sensor_data', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
