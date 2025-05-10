#!/usr/bin/env python3

import rospy
import math
import numpy as np
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64

class StanleyControllerWithObstacleAvoidance:
    def __init__(self):
        rospy.init_node('Autonomous_Systems_MS_4_CLR_Alg2_Lateral_Team_08')
        rospy.loginfo("The Node has been started\n")

        # In __init__():
        self.desired_lane = 2.5  # Initial lane
        rospy.Subscriber('/desired_lane', Float64, self.lane_callback)
        
        # Controller parameters
        self.k = rospy.get_param('~k', 0.5) # Gain for cross-track error
        self.epsilon = rospy.get_param('~epsilon', 0.01) # Small value to prevent division by zero
        self.max_steer = rospy.get_param('~max_steer', 0.4) # Maximum steering angle (in radians)
        self.desired_lane = rospy.get_param('~desired_lane', 3.5) # Desired lane position (in meters)

        # Smoothing filter parameters
        self.steering_filter_alpha = 0.5  # Smoothing factor (0.0-1.0, higher = smoother)
        self.previous_steering = 0.0

        # Obstacle detection parameters (simulate obstacle positions)
        self.obstacle_left = 250.0  # Position of an obstacle on the left (in meters)
        self.obstacle_right = 500.0  # Position of an obstacle on the right (in meters)

        # Vehicle state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_speed = 0.0
        self.prev_pose = None
        self.prev_time = None

        # ROS infrastructure
        self.steer_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=10)
        rospy.Subscriber('/filtered_pose', Pose2D, self.pose_callback)
        
        rospy.loginfo(f"Stanley Lateral Controller Initialized | Desired Lane: {self.desired_lane:.2f}m")

    def lane_callback(self, msg):
        # Update the desired lane from the topic
        self.desired_lane = msg.data
        rospy.loginfo(f"Desired Lane Updated: {self.desired_lane:.2f}m")

    def pose_callback(self, msg):
        """Handle filtered pose updates and calculate speed"""
        current_time = rospy.Time.now()
        
        # Update position and orientation
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_yaw = msg.theta

        # Calculate speed using pose changes
        if self.prev_pose is not None and self.prev_time is not None:
            dt = (current_time - self.prev_time).to_sec()
            if dt > 0:
                dx = msg.x - self.prev_pose.x
                dy = msg.y - self.prev_pose.y
                # Calculate speed in vehicle's forward direction
                self.current_speed = (dx * np.cos(msg.theta)) + (dy * np.sin(msg.theta))
                self.current_speed /= dt

        self.prev_pose = msg
        self.prev_time = current_time

        # Update obstacle distances dynamically
        self.update_obstacle_distances()
        # Run control logic
        self.check_obstacles_and_steer()

    def update_obstacle_distances(self):
        # Placeholder for dynamically updating obstacle distances
        self.obstacle_left = rospy.get_param('~obstacle_left', self.obstacle_left)
        self.obstacle_right = rospy.get_param('~obstacle_right', self.obstacle_right)

    def check_obstacles_and_steer(self):
        # Determine which side to avoid based on obstacles
        if self.current_speed > 0:  # Only check obstacles if moving
            # Check left obstacle proximity
            if (7.0 > self.current_y > 0.0 and 
                self.obstacle_left - self.current_x < 100.0 and 
                self.obstacle_left - self.current_x > 0.0):
                rospy.loginfo(f"Obstacle detected in left lane at {self.obstacle_left}m. Moving right.")
                self.desired_lane = -2.5

            # Check right obstacle proximity
            elif (-7.0 < self.current_y < 0.0 and 
                  self.obstacle_right - self.current_x < 100.0 and 
                  self.obstacle_right - self.current_x > 0.0):
                rospy.loginfo(f"Obstacle detected in right lane at {self.obstacle_right}m. Moving left.")
                self.desired_lane = 2.5

        # Compute and smooth steering
        steering, e, heading_error = self.compute_steering()
        steering = (self.steering_filter_alpha * steering + 
                   (1 - self.steering_filter_alpha) * self.previous_steering)
        steering = max(-self.max_steer, min(steering, self.max_steer))
        
        # Publish command
        self.steer_pub.publish(Float64(steering))
        self.previous_steering = steering

    def compute_steering(self):
        # Calculate cross-track error
        e = self.desired_lane - self.current_y
        
        # Prevent division by zero
        effective_speed = max(abs(self.current_speed), self.epsilon)
        
        # Stanley control law components
        heading_error = -self.current_yaw  # Negative because yaw is positive CCW
        cross_track_term = math.atan2(self.k * e, effective_speed)
        
        # Combined steering command
        steering = 0.8 * heading_error + 1.2 * cross_track_term
        return steering, e, heading_error

if __name__ == '__main__':
    try:
        StanleyControllerWithObstacleAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")