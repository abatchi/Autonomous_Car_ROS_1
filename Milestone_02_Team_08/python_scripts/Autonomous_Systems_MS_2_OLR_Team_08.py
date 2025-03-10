#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, UInt8
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState


def odom_callback(msg):
    rospy.loginfo(f"Odom Position: {msg.pose.pose.position}")
    rospy.loginfo(f"Odom Orientation: {msg.pose.pose.orientation}")
    rospy.loginfo(f"Odom Twist: {msg.twist.twist}")


def steering_callback(msg):
    rospy.loginfo(f"Steering Angle: {msg.data}")


def model_states_callback(msg):
    rospy.loginfo(f"Model States: {msg.pose}")


def move_audibot():
    rospy.init_node("Autonomous_Systems_MS_2_OLR_Team_08")

    # Publishers for throttle, steering, brake, and gear
    throttle_pub = rospy.Publisher("/throttle_cmd", Float64, queue_size=10)
    steering_pub = rospy.Publisher("/steering_cmd", Float64, queue_size=10)
    brake_pub = rospy.Publisher("/brake_cmd", Float64, queue_size=10)
    gear_pub = rospy.Publisher("/gear_cmd", UInt8, queue_size=10)
    
    # Subscribers for odom and model states
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/steering_state", Float64, steering_callback)
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)

    rospy.sleep(2)  # Give time for publishers to register

    # Setting gear to DRIVE and ensuring brake is zero
    gear_pub.publish(UInt8(0))  # Set gear to DRIVE (0 is usually for forward drive in most systems)
    brake_pub.publish(Float64(0.0))  # Ensure brake is released (0.0 for no brake)

    rate = rospy.Rate(10)  # 10 Hz loop rate

    steering_angle = 0.0
    throttle_value = 0.1  # Starting throttle value

    while not rospy.is_shutdown():
        rospy.loginfo(f"Moving with steering angle: {steering_angle} and throttle: {throttle_value}")
        
        # Incrementally adjust steering and throttle
        steering_angle += 0.01
        if steering_angle > 1:
            steering_angle = 1  # Limit max steering angle
        
        throttle_value += 0.01
        if throttle_value > 0.7:
            throttle_value = 0.7  # Limit max throttle
        
        # Publishing commands
        steering_pub.publish(Float64(steering_angle))
        throttle_pub.publish(Float64(throttle_value))
        
        # Sleep for a short duration to maintain loop rate
        rospy.sleep(0.1)
    
    rospy.loginfo("Stopping the car")
    throttle_pub.publish(Float64(0.0))  # Stop the car when done
    
    rospy.spin()

if __name__ == "__main__":
    try:
        move_audibot()
    except rospy.ROSInterruptException:
        pass
