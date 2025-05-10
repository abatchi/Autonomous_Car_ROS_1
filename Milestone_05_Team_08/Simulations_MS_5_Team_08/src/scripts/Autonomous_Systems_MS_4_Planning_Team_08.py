#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import ast 

class PathPlanner:
    def __init__(self):
        rospy.init_node('Autonomous_Systems_MS_4_Planning_Team_08')
        rospy.loginfo("The Node has been started\n")


        # Maintain your original lane values
        self.lane_1 = rospy.get_param('~lane_1', 2.5)   # Left lane (keep original value)
        
        self.lane_2 = rospy.get_param('~lane_2', -2.5)   # Right lane (keep original value)

        self.lane_3 = rospy.get_param('~lane_3', 0.0)   # Center lane (keep original value)

        self.obstacle_positions = rospy.get_param('~obstacle_positions', [250.0, 500.0])  # Simulated obstacle positions (in meters)
        self.obstacle_positions = ast.literal_eval(self.obstacle_positions)
        self.obstacle_positions = [float(x) for x in self.obstacle_positions]

        # Start changing lane 50m before obstacle
        self.lane_change_distance = rospy.get_param('~lane_change_distance', 50.0) # 50.0 m 
        # Velocity profile (maintain your speed parameters)
        self.normal_speed = rospy.get_param('~normal_speed', 10.0)   # 10.0 m/s
        self.lane_change_speed = rospy.get_param('~lane_change_speed', 6.0) # 6.0 m/s

        
        # State tracking
        self.current_x = 0.0
        self.current_lane = self.lane_1  # Start in the center lane
        self.current_speed = 0.0         
        self.odom_received = False 
        self.current_y = 0.0 
        self.current_yaw = 0.0
        self.obstacle_index = 0

        rospy.loginfo(type(self.obstacle_positions[self.obstacle_index]))

        # ROS interfaces
        self.speed_pub = rospy.Publisher('/desired_speed', Float64, queue_size=10) 
        self.lane_pub = rospy.Publisher('/desired_lane', Float64, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.sleep(0.1)   # let pubs register


        self.lane_pub.publish(Float64(self.current_lane))   # center lane = lane_3 = 0.0
        self.speed_pub.publish(Float64(self.normal_speed))


    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = msg.pose.pose.orientation.z
        self.current_speed = msg.twist.twist.linear.x
        self.odom_received = True
        
        if self.obstacle_index < len(self.obstacle_positions):
            obstacle_x = self.obstacle_positions[self.obstacle_index]
            
            if self.current_x > obstacle_x - self.lane_change_distance:
                # Toggle lane while maintaining your original values
                new_lane = self.lane_2 if self.current_lane == self.lane_1 else self.lane_1
                self.lane_pub.publish(Float64(new_lane))
                self.speed_pub.publish(Float64(self.lane_change_speed))
                self.current_lane = new_lane
                self.obstacle_index += 1
            else:
                self.speed_pub.publish(Float64(self.normal_speed))
        else:
            self.speed_pub.publish(Float64(self.normal_speed))

if __name__ == '__main__':
    PathPlanner()
    rospy.spin()