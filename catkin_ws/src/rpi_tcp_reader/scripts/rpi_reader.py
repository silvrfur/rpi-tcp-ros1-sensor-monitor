#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import socket

def tcp_listener():
    rospy.init_node('rpi_tcp_reader', anonymous=True)
    pub = rospy.Publisher('/sensor_data', String, queue_size=10)

    HOST = '0.0.0.0'  # listen on all interfaces
    PORT = 5005

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    rospy.loginfo("Waiting for Raspberry Pi connection...")

    conn, addr = server_socket.accept()
    rospy.loginfo(f"Connected to Raspberry Pi: {addr}")

    try:
        while not rospy.is_shutdown():
            data = conn.recv(1024).decode().strip()
            if data:
                rospy.loginfo(f"Received: {data}")
                pub.publish(data)
    except Exception as e:
        rospy.logerr(f"Error: {e}")
    finally:
        conn.close()
        server_socket.close()

if __name__ == '__main__':
    try:
        tcp_listener()
    except rospy.ROSInterruptException:
        pass
