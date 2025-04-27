#!/usr/bin/env python2

import rospy

def validation_node():
    rospy.init_node('validation_printing_node', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        rospy.loginfo("Validation Printing Node is running...")
        rate.sleep()

if __name__ == '__main__':
    try:
        validation_node()
    except rospy.ROSInterruptException:
        pass

