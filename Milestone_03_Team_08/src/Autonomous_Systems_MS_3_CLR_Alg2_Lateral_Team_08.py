#!/usr/bin/env python3

import rospy
import math
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class StanleyController:
    def __init__(self):
        rospy.init_node('Autonomous_Systems_MS_3_CLR_Alg2_Lateral_Team_08')
        
        # Controller parameters
        self.k = rospy.get_param('~k', 20)
        self.epsilon = rospy.get_param('~epsilon', 0.01)
        self.max_steer = rospy.get_param('~max_steer', 1.5)
        self.desired_lane = rospy.get_param('~desired_lane', 2.0)

        # Vehicle state
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_speed = 0.0

        # ROS infrastructure
        self.steer_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        rospy.loginfo(f"Stanley Lateral Controller Initialized | Desired Lane: {self.desired_lane:.2f}m")

    def odom_callback(self, msg):
        # Extract position and orientation
        self.current_y = msg.pose.pose.position.y
        self.current_speed = msg.twist.twist.linear.x
        
        # Convert quaternion to yaw
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, self.current_yaw = tf.transformations.euler_from_quaternion(quaternion)

        # Compute control
        steering, e, heading_error = self.compute_steering()
        self.steer_pub.publish(Float64(steering))

        # Continuous status logging
        rospy.loginfo(f"[Lateral Ctrl] Desired Lane: {self.desired_lane:.2f}m | " 
                      f"Cross-Track Error: {e:.2f}m | "
                      f"Heading Error: {math.degrees(heading_error):.1f}° | "
                      f"Steering: {math.degrees(steering):.1f}°")

    def compute_steering(self):
        # Calculate cross-track error
        e = self.desired_lane - self.current_y
        
        # Prevent division by zero
        effective_speed = max(abs(self.current_speed), self.epsilon)
        
        # Stanley control law components
        heading_error = -self.current_yaw  # Negative because yaw is positive CCW
        cross_track_term = math.atan2(self.k * e, effective_speed)
        
        # Final steering angle
        steering = heading_error + cross_track_term
        steering = max(-self.max_steer, min(steering, self.max_steer))

        return steering, e, heading_error

if __name__ == '__main__':
    try:
        StanleyController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass