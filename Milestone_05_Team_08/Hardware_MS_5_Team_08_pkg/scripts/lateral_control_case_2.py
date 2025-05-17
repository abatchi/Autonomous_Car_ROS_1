#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float64, Float64MultiArray
global heading_rad
heading_rad = 0.0
class StanleyLateralController:
    def __init__(self):
        rospy.init_node('stanley_lateral_controller', anonymous=True)
        rospy.loginfo("Stanley Lateral Controller Node Started...")

        # Stanley controller parameters (fixed values)
        self.k = 2 # Control gain
        self.speed = 5.0  # Assume constant speed 1 m/s
        self.heading_error = 0.2
        self.cross_track_error = 0.5  # Assume fixed cross-track error for now

        # Subscriber: Subscribe to IMU data (from pub2 Float64MultiArray)
        rospy.Subscriber('imu_data', Float64, self.imu_callback)
        rospy.Subscriber('desired_heading',Float64,self.heading_callback)

        # Publisher: Publish to steering command
        self.steering_pub = rospy.Publisher('steering_cmd', Float64, queue_size=10)
        

        rospy.spin()
    def heading_callback(self,msg):
        global heading_deg,heading_rad
        heading_deg = float(msg.data)
        heading_rad = math.radians(heading_deg)
    def imu_callback(self, msg):
        # 1) Read yaw angle (°) from msg
        try:
            yaw_deg = float(msg.data)
        except (TypeError, ValueError):
            rospy.logwarn("Invalid IMU yaw data.")
            return

        # 2) Convert yaw to radians and set as heading error
        yaw_rad = math.radians(yaw_deg)
        self.heading_error = yaw_rad - heading_rad 

        # 3) Stanley control law → steering angle in radians
        steering_rad = self.heading_error + math.atan2(self.k * self.cross_track_error, self.speed)

        # 4) Clamp to ±30°
        max_rad = math.radians(30)
        steering_rad = max(-max_rad, min(max_rad, steering_rad))

        # 5) Convert to degrees
        steering_deg = math.degrees(steering_rad)

        # 6) Map [–30, +30] → [servo_left, servo_right] = [90, 140]
        #    with 0° → 120
        servo_left  =  90
        servo_zero  = 120
        servo_right = 140

        # linear map: steering_deg of –30 → servo_left ; +30 → servo_right
        # formula: servo_cmd = (steering_deg + 30) / 60 * (servo_right - servo_left) + servo_left
        servo_cmd = int((steering_deg + 30.0) / 60.0 * (servo_right - servo_left) + servo_left)

        # clamp just in case
        servo_cmd = max(servo_left, min(servo_right, servo_cmd))

        # 7) Publish the servo command
        self.steering_pub.publish(servo_cmd)

        rospy.loginfo(f"Steering Cmd: {servo_cmd} (deg ctrl: {steering_deg:.1f}°)")


if __name__ == '__main__':
    try:
        StanleyLateralController()
    except rospy.ROSInterruptException:
        pass



