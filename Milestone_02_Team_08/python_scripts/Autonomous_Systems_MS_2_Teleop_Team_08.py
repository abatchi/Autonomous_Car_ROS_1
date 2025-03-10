#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, UInt8
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import sys
import termios
import tty
import select

# Function to get keyboard input
def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
    return key

# Callback functions for subscribers
def odom_callback(msg):
    rospy.loginfo(f"Odom Position: {msg.pose.pose.position}")

def steering_callback(msg):
    rospy.loginfo(f"Steering Angle: {msg.data}")

def model_states_callback(msg):
    rospy.loginfo(f"Model States: {msg.pose}")

def move_audibot():
    rospy.init_node("Autonomous_Systems_MS_2_Teleop_Team_08")
    
    rospy.loginfo("The Node has been started")

    throttle_pub = rospy.Publisher("/throttle_cmd", Float64, queue_size=10)
    steering_pub = rospy.Publisher("/steering_cmd", Float64, queue_size=10)
    brake_pub = rospy.Publisher("/brake_cmd", Float64, queue_size=10)
    gear_pub = rospy.Publisher("/gear_cmd", UInt8, queue_size=10)
    
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/steering_state", Float64, steering_callback)
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
    
    rospy.sleep(2)  # Give time for publishers to register
    
    gear_pub.publish(UInt8(0))  # Set gear to DRIVE
    
    rate = rospy.Rate(10)

    braking_value = 0.0
    steering_angle = 0.0
    throttle_value = 0.0
    gear = 0

    # Decay factors for throttle and steering
    throttle_decay = 0.95  # Adjust this value to control deceleration rate
    steering_decay = 0.95  # Adjust this value to control steering return rate

    rospy.loginfo("Use Arrow Keys to Control the Car. Press 'q' to Quit.")

    while not rospy.is_shutdown():
        key = get_key()

        if key == '\x1b':  # Detect arrow keys (Escape sequence)
            key = sys.stdin.read(2)  # Read next two characters
            if key == '[A':  # Up Arrow
                braking_value = 0
                gear_pub.publish(UInt8(0))
                throttle_value += 0.02
                if throttle_value > 0.7:
                    throttle_value = 0.7  # Limit throttle

            elif key == '[B':  # Down Arrow
                gear_pub.publish(UInt8(1))  # Set gear to Reverse
                braking_value = 0
                throttle_value += 0.02
                if throttle_value < 0.7:
                    throttle_value = 0.7  # Limit throttle

            elif key == '[C':  # Right Arrow
                steering_angle -= 0.1
                if steering_angle < -1.5:
                    steering_angle = -1.5  # Limit steering right

            elif key == '[D':  # Left Arrow
                steering_angle += 0.1
                if steering_angle > 1.5:
                    steering_angle = 1.5  # Limit steering left

        elif key == ' ':  # Spacebar (Stop)
            throttle_value = 0.0

        elif key == 'q':  # Quit
            rospy.loginfo("Exiting...")
            break
        
        if key == 'b': #braking
            braking_value += 100.0
            if braking_value > 1000.0:
                braking_value = 1000.0

        # Apply decay to throttle and steering when no key is pressed
        if key == '':
            throttle_value *= throttle_decay
            steering_angle *= steering_decay

        # Publish values
        steering_pub.publish(Float64(steering_angle))
        throttle_pub.publish(Float64(throttle_value))
        brake_pub.publish(Float64(braking_value))
        
        rospy.loginfo(f"Throttle: {throttle_value}, Steering: {steering_angle}, Gear: {gear}")
        
        rate.sleep()

    # Stop the car when exiting
    throttle_pub.publish(Float64(0.0))
    steering_pub.publish(Float64(0.0))
    rospy.loginfo("Car stopped.")

if __name__ == "__main__":
    try:
        move_audibot()
    except rospy.ROSInterruptException:
        pass