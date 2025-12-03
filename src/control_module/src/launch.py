#!/usr/bin/env python

import json
import os
import socket

import rospy
from std_msgs.msg import String


def load_config():
    """Load TCP server configuration."""
    default_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "config", "control_config.json")
    config_path = os.environ.get("CONTROL_CONFIG_PATH", default_path)

    if not os.path.exists(config_path):
        raise RuntimeError(
            f"Control config file not found: {config_path}. "
            "Copy control_config.json.example and set CONTROL_CONFIG_PATH if needed."
        )

    with open(config_path, "r") as f:
        data = json.load(f)

    required = ["host", "port", "buffer_size"]
    missing = [k for k in required if k not in data or data[k] in (None, "")]
    if missing:
        raise RuntimeError(f"Control config missing keys: {missing}")

    data["port"] = int(data["port"])
    data["buffer_size"] = int(data["buffer_size"])
    return data


def start_tcp_server():
    cfg = load_config()
    rospy.init_node("control_node", anonymous=True)
    sensor_pub = rospy.Publisher("sensor_command", String, queue_size=10)

    host = cfg["host"]
    port = cfg["port"]
    buffer_size = cfg["buffer_size"]

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
                data = client_socket.recv(buffer_size).decode("utf-8")
                if not data:
                    break

                partial_data += data
                commands = partial_data.split("\n")
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


if __name__ == "__main__":
    try:
        start_tcp_server()
    except rospy.ROSInterruptException:
        pass
