#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import serial
import time

def setup_serial():
    try:
        arduino = serial.Serial('/dev/ttyUSB0', 9600)
        time.sleep(2)  # Allow time for Arduino to reset
        rospy.loginfo("Connected to Arduino on /dev/ttyUSB0")
        return arduino
    except serial.SerialException as e:
        rospy.logerr(f"Serial error: {e}")
        return None

def callback(msg):
    command = msg.data
    print("Sent to Arduino:", command)
    if arduino:
        arduino.write((command + "\n").encode())

if __name__ == "_main_":
    rospy.init_node("command_receiver_node")
    arduino = setup_serial()
    rospy.Subscriber("/control_command", String, callback)
    rospy.spin()