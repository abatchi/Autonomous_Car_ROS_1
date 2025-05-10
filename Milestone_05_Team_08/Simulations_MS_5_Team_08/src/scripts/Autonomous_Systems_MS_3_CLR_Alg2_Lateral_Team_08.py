#!/usr/bin/env python3

import rospy
import math
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D


class StanleyController:
    def __init__(self):
        rospy.init_node('Autonomous_Systems_MS_3_CLR_Alg2_Lateral_Team_08')
        rospy.loginfo("Lateral Control Node Initialized\n")

        # Controller parameters
        self.k = rospy.get_param('~k', 1.2)  # Tuned cross-track gain
        self.epsilon = rospy.get_param('~epsilon', 0.01)
        self.max_steer = rospy.get_param('~max_steer', 0.35)  # Reduced max steering
        self.desired_lane = rospy.get_param('~desired_lane', 0.0)   # Center lane

        self.lane_sub = rospy.Subscriber('/desired_lane', Float64, self.lane_callback)

        # Steering smoothing
        self.steering_filter_alpha = rospy.get_param('~steering_filter_alpha' , 0.85)  # Stronger smoothing
        self.previous_steering = 0.0

        self.kd_lateral = rospy.get_param('~kd_lateral', 0.1) # Lateral gain
        self.prev_cte = 0.0

        # Vehicle state
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_speed = 0.0

        # ROS interfaces
        self.steer_pub = rospy.Publisher('/steering_cmd', Float64, queue_size=10)
        rospy.Subscriber('/filtered_pose', Pose2D, self.filtered_callback)


    def lane_callback(self, msg):
        """Handle new lane position updates from planning node"""
        self.desired_lane = msg.data
        rospy.logdebug(f"New target lane: {self.desired_lane:.2f}m")

    def filtered_callback(self, msg):
        self.current_y = msg.y
        self.current_yaw = msg.theta
        # Use self.current_speed from /twist if needed, or keep from /odom if confident
        steering, e, heading_error = self.compute_steering()

        smoothed_steering = (self.steering_filter_alpha * self.previous_steering +
                            (1 - self.steering_filter_alpha) * steering)

        final_steering = max(-self.max_steer, min(smoothed_steering, self.max_steer))
        self.steer_pub.publish(Float64(final_steering))
        self.previous_steering = final_steering

        # rospy.loginfo_throttle(1,
        #     f"[Lateral] Lane: {self.desired_lane:.2f}m | "
        #     f"CTE: {e:.2f}m | "
        #     f"Heading Err: {math.degrees(heading_error):.1f}° | "
        #     f"Steering: {math.degrees(final_steering):.1f}°"
        # )


    def compute_steering(self):
        """Stanley control law implementation"""
        cte = self.desired_lane - self.current_y
        effective_speed = max(abs(self.current_speed), self.epsilon)
        
        # Heading error compensation
        heading_error = -self.current_yaw  # Account for yaw direction convention
        
        # Cross-track term with soft saturation
        cross_track_term = math.atan((self.k * cte) / (effective_speed + 1.0))
        
        # Blended control law
        steering = 0.6 * heading_error + 1.4 * cross_track_term
        return steering, cte, heading_error

if __name__ == '__main__':
    try:
        StanleyController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Controller Error: {str(e)}")