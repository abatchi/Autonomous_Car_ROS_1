#!/usr/bin/env python3

import rospy
import pygame
from std_msgs.msg import String
import time

def init_controller():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        rospy.logerr("No controller detected.")
        exit()

    controller = pygame.joystick.Joystick(0)
    controller.init()
    rospy.loginfo(f"Controller connected: {controller.get_name()}")
    return controller

def main():
    rospy.init_node("controller_logic_node")
    pub = rospy.Publisher("/control_command", String, queue_size=10)

    controller = init_controller()
    last_command = ""
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        pygame.event.pump()

        A = controller.get_axis(0)
        right_pressed = controller.get_button(5)
        
        
        if A < 0:
            command = "LEFT"
            if right_pressed:
                command = "LEFTMOVE"
        elif A > 0:
            command = "RIGHT"
            if right_pressed:
                command = "RIGHTMOVE"
        else:
            command = "CENTER"
            if right_pressed:
                command = "MOVE"

        if command != last_command:
            pub.publish(command)
            last_command = command

        rate.sleep()

if __name__ == "_main_":
    main()