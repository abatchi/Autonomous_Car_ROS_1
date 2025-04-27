#!/usr/bin/env python3

import rospy
import math
import tf
from nav_msgs.msg import Odometry
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

        # ROS infrastructure
        self.steer_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        rospy.loginfo(f"Stanley Lateral Controller Initialized | Desired Lane: {self.desired_lane:.2f}m")


    def lane_callback(self, msg):
        # Update the desired lane from the topic
        self.desired_lane = msg.data
        rospy.loginfo(f"Desired Lane Updated: {self.desired_lane:.2f}m")


    def odom_callback(self, msg):
        # Extract position and orientation
        self.current_y = msg.pose.pose.position.y
        self.current_x = msg.pose.pose.position.x
        self.current_speed = msg.twist.twist.linear.x
        
        # Convert quaternion to yaw
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, self.current_yaw = tf.transformations.euler_from_quaternion(quaternion)

        # Update obstacle distances dynamically (placeholder for sensor data)
        self.update_obstacle_distances()

        # Determine if an obstacle is in the way
        self.check_obstacles_and_steer()

    def update_obstacle_distances(self):
        # Placeholder for dynamically updating obstacle distances
        # Replace these with actual sensor data or simulation inputs
        self.obstacle_left = rospy.get_param('~obstacle_left', self.obstacle_left)
        self.obstacle_right = rospy.get_param('~obstacle_right', self.obstacle_right)

    def check_obstacles_and_steer(self):


        # Determine which side to avoid based on obstacles
        if self.current_speed > 0:  # Only check obstacles if the vehicle is moving
            # Check if the vehicle is within the blocked range of the left obstacle
            if self.current_y < 7.0 and self.current_y > 0.0 and self.obstacle_left - self.current_x < 100.0 and self.obstacle_left - self.current_x > 0.0:
                # If the left lane is blocked, switch to the right lane
                # Check if the vehicle is within the blocked range of the right obstacle
                # If the right lane is blocked, switch to the left lane
                rospy.loginfo(f"Obstacle detected in left lane at {self.obstacle_left}m. Moving to the right lane.")
                self.desired_lane = -2.5  # Switch to the right lane

            # Check if the vehicle is within the blocked range of the right obstacle
            elif self.current_y > -7.0 and self.current_y < 0.0 and self.obstacle_right - self.current_x  < 100.0 and self.obstacle_right - self.current_x > 0.0:
                # If the right lane is blocked, switch to the left lane 
                # Check if the vehicle is within the blocked range of the left obstacle
                # If the left lane is blocked, switch to the right lane
                rospy.loginfo(f"Obstacle detected in right lane at {self.obstacle_right}m. Moving to the left lane.")
                self.desired_lane = 2.5  # Switch to the left lane

        # Compute steering after determining the desired lane
        self.steering, e, heading_error = self.compute_steering()
        # Apply smoothing filter to the steering command
        self.steering = (self.steering_filter_alpha * self.steering + 
                         (1 - self.steering_filter_alpha) * self.previous_steering) 
        # Apply a simple low-pass filter to smooth the steering command
        # self.steering = 0.8 * self.steering + 0.2 * self.previous_steering
        # Update the previous steering command for the next iteration
        self.previous_steering = self.steering  # Update previous steering for next iteration
        # Limit the steering angle to the maximum allowed
        self.steering = max(-self.max_steer, min(self.steering, self.max_steer))
        # Publish the steering command
        self.steer_pub.publish(Float64(self.steering))

        # Continuous status logging
        rospy.loginfo(f"[Lateral Ctrl] Desired Lane: {self.desired_lane:.2f}m | " 
                    f"Cross-Track Error: {e:.2f}m | "
                    f"Heading Error: {math.degrees(heading_error):.1f}° | "
                    f"Steering: {math.degrees(self.steering):.1f}°")

    def compute_steering(self):
        # Calculate cross-track error
        e = self.desired_lane - self.current_y
        
        # Prevent division by zero
        effective_speed = max(abs(self.current_speed), self.epsilon)
        
        # Stanley control law components
        heading_error = -self.current_yaw  # Negative because yaw is positive CCW
        cross_track_term = math.atan2(self.k * e, effective_speed) # Cross-track term
        
        # Final steering angle
        steering = 0.8 * heading_error + 1.2 * cross_track_term  # Tuned weights
        steering = max(-self.max_steer, min(steering, self.max_steer))

        return steering, e, heading_error

if __name__ == '__main__':
    try:
        StanleyControllerWithObstacleAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
