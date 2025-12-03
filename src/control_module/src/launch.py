#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import socket

def start_tcp_server():
    rospy.init_node('control_node', anonymous=True)
    sensor_pub = rospy.Publisher('sensor_command', String, queue_size=10)

    host = '0.0.0.0'
    port = 5000
    buffer_size = 1024

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(5)
    rospy.loginfo(f"TCP Server started on {host}:{port}")
    rospy.loginfo("Waiting for a connection...")

    while not rospy.is_shutdown():
        client_socket, client_address = server_socket.accept()
        rospy.loginfo(f"Connection from {client_address}")

        try:
            partial_data = ""
            while not rospy.is_shutdown():
                data = client_socket.recv(buffer_size).decode('utf-8')
                if not data:
                    break

                partial_data += data
                commands = partial_data.split('\n')
                partial_data = commands[-1]

                for command in commands[:-1]:
                    command = command.strip()
                    if command:
                        rospy.loginfo(f"Received: {command}")

                        if "sensor" in command:
                            sensor_pub.publish(command)

        except Exception as e:
            rospy.logerr(f"Error: {e}")

        finally:
            client_socket.close()
            rospy.loginfo("Connection closed")

if __name__ == '__main__':
    try:
        start_tcp_server()
    except rospy.ROSInterruptException:
        pass
