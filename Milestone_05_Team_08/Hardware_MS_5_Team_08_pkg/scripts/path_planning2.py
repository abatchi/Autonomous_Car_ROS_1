#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import time
import math
yaw = 0.0
actual_speed = 0.0
speed_updated = False
global x,y,yaw_rad
global flag1,flag2,flag3,flag4,flag5

# flag6 = True
# flag7 =True

def imu_case_2_callback(msg):
    global yaw
    yaw = msg.data
    yaw = abs(yaw)
    
    rospy.loginfo("Yaw: %.2f m/s", yaw)
    
    
def actual_callback(msg):
    global actual_speed, speed_updated
    actual_speed = msg.data
    speed_updated = True
    rospy.loginfo("actual speed: %.2f m/s", actual_speed)
    
    
def path_planner_case_2():
    rospy.init_node('simple_path_planner')
    pub = rospy.Publisher('/desired_speed', Float64, queue_size=10)
    pub2 = rospy.Publisher('/desired_heading', Float64, queue_size=10)
    rospy.Subscriber('/actual', Float64, actual_callback)
    rospy.Subscriber('/imu_data', Float64, imu_case_2_callback)
    try:
        desired_speed = 1.28
        
    except ValueError:
        rospy.logerr('Invalid input. Please enter a number.')
        return
    rate = rospy.Rate(10)  # 10 Hz
    traveled_distance = 0.0
    prev_time = rospy.get_time()
    prev_speed = 0.0
    start_time = time.time()
    yaw_rad = math.radians(yaw)
    flag1 =True
    flag2 = True
    flag3 =True
    flag4 = True
    flag5 = True
    while not rospy.is_shutdown():
        yaw_rad = math.radians(yaw)
        current_time = rospy.get_time()
        dt = current_time - prev_time
        
        if dt <= 0:
            rate.sleep()
            
            continue
        
        avg_speed = 0.5*(prev_speed + actual_speed)
        delta_distance = avg_speed * dt
        traveled_distance += delta_distance
        x = traveled_distance* math.cos(yaw_rad)
        y = traveled_distance* math.sin(yaw_rad)

        #elapsed_time = time.time() - start_time
        #traveled_distance = (actual_speed * elapsed_time) + traveled_distance
        if x < 1400.0:  # the scale 1400 corresponds to 10 meters aprrox
            # Calculate the desired heading based on the current position
            if x <= 50 :
                heading = 30.0
#             elif y <= 50.0 and y>=15 and x < 560.0 and flag2:
#                 heading = 0.0
            elif x>=300.0 and x < 400.0:
                heading = -20.0
#             elif y>= 72.0 and  y<=65.0 and x>=560.0 and x< 900.0 and flag4:
#                 heading = 0.0
            elif x >= 800.0 and x<900.0:
                heading = 30.0
            else:
                heading = 10.0
                
                
            rospy.loginfo('Distance: %.2f m - Driving at %.2f m/s', traveled_distance, desired_speed)
            pub.publish(Float64(desired_speed))
            pub2.publish(Float64(heading))
            
        else:
            rospy.loginfo('Reached 10 meters. Stopping the car.')
            pub.publish(Float64(0.0))
            pub2.publish(Float64(0.0))
            break
        #prev_time = current_time
        #prev_speed = actual_speed
        rate.sleep()
        rospy.loginfo('x: %.2f m - y : %.2f m', x, y)
        rospy.loginfo('heading: %.2f deg', heading)

if __name__ == '__main__':
    try:
        path_planner_case_2()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass




