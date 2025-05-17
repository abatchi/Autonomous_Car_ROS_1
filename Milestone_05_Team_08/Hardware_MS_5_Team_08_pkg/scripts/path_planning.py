#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import time
actual_speed = 0.0
speed_updated = False
def actual_callback(msg):
        global actual_speed, speed_updated
        actual_speed = msg.data
        speed_updated = True
        rospy.loginfo("actual speed: %.2f m/s", actual_speed)
def simple_path_planner():
    rospy.init_node('simple_path_planner')
    pub = rospy.Publisher('/desired_speed', Float64, queue_size=10)
    rospy.Subscriber('/actual', Float64, actual_callback)
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
    while not rospy.is_shutdown():
        current_time = rospy.get_time()
        dt = current_time - prev_time
        
        if dt <= 0:
            rate.sleep()
            
            continue
        
        avg_speed = 0.5*(prev_speed + actual_speed)
        delta_distance = avg_speed * dt
        traveled_distance += delta_distance
        #elapsed_time = time.time() - start_time
        #traveled_distance = (actual_speed * elapsed_time) + traveled_distance
        if traveled_distance < 1400.0:
            rospy.loginfo('Distance: %.2f m - Driving at %.2f m/s', traveled_distance, desired_speed)
            pub.publish(Float64(desired_speed))
        else:
            rospy.loginfo('Reached 10 meters. Stopping the car.')
            pub.publish(Float64(0.0))
            break
        #prev_time = current_time
        #prev_speed = actual_speed
        rate.sleep()

if __name__ == '__main__':
    try:
        simple_path_planner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



