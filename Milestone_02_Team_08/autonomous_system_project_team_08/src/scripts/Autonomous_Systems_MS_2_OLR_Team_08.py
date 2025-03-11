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

    # Retrieve parameters from the launch file
    initial_throttle = rospy.get_param("~initial_throttle", 0.1)
    initial_steering = rospy.get_param("~initial_steering", 0.0)
    throttle_step = rospy.get_param("~throttle_step", 0.01)
    steering_step = rospy.get_param("~steering_step", 0.01)
    max_throttle = rospy.get_param("~max_throttle", 0.7)
    max_steering = rospy.get_param("~max_steering", 1.5)

    rospy.loginfo(f"Initial Throttle: {initial_throttle}, Initial Steering: {initial_steering}")
    rospy.loginfo(f"Throttle Step: {throttle_step}, Steering Step: {steering_step}")
    rospy.loginfo(f"Max Throttle: {max_throttle}, Max Steering: {max_steering}")

    # Publishers
    throttle_pub = rospy.Publisher("/throttle_cmd", Float64, queue_size=10)
    steering_pub = rospy.Publisher("/steering_cmd", Float64, queue_size=10)
    brake_pub = rospy.Publisher("/brake_cmd", Float64, queue_size=10)
    gear_pub = rospy.Publisher("/gear_cmd", UInt8, queue_size=10)

    rospy.sleep(2)  # Give time for publishers to register

    brake_value = 0.0
    gear_pub.publish(UInt8(0))  # Set gear to DRIVE
    brake_pub.publish(brake_value)  # Ensure brake is released

    rate = rospy.Rate(10)  # 10 Hz loop rate

    steering_angle = initial_steering
    throttle_value = initial_throttle

    while not rospy.is_shutdown():
        rospy.loginfo(f"Moving with steering angle: {steering_angle} and throttle: {throttle_value}")

        # Incrementally adjust steering and throttle
        throttle_value += throttle_step
        if throttle_value > max_throttle:
            throttle_value = max_throttle  # Limit max throttle

        steering_angle += steering_step
        if abs(steering_angle) > max_steering:
            steering_angle = max_steering if steering_angle > 0 else -max_steering  # Limit max steering
        
        # Publish commands
        steering_pub.publish(Float64(steering_angle))
        throttle_pub.publish(Float64(throttle_value))
        brake_pub.publish(Float64(brake_value))  # Ensure brake is released
        
        rospy.sleep(0.1)
    
    rospy.loginfo("Stopping the car")
    throttle_pub.publish(Float64(0.0))  # Stop the car when done
    
    rospy.spin()


if __name__ == "__main__":
    try:
        move_audibot()
    except rospy.ROSInterruptException:
        pass
