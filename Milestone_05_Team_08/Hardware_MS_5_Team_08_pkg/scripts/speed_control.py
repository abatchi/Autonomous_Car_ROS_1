#!/usr/bin/env python3

import rospy
import threading
import time
from std_msgs.msg import String, Float64

class PIDController:
    def __init__(self, Kp, Ki, Kd, min_output, max_output):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.min_output = min_output
        self.max_output = max_output
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = rospy.Time.now()

    def compute(self, error):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        if dt <= 0:
            return 0.0

        P = self.Kp * error
        self.integral += error * dt
        I = self.Ki * self.integral
        D = self.Kd * (error - self.previous_error) / dt

        output = P + I + D
        output = max(self.min_output, min(output, self.max_output))

        self.previous_error = error
        self.last_time = current_time

        return output

class RealCarSpeedController:
    
    def __init__(self):
        rospy.init_node('real_car_speed_controller')
        rospy.loginfo("Real Car Speed Controller Node Started...")

        # Parameters
        Kp = 2
        Ki = 0
        Kd = 0.01
        min_throttle = 0.5
        max_throttle = 1

        self.desired_speed = 0.0
        self.current_speed = 0.0
        self.last_ticks = 0.0
        # Encoder config
        self.last_rev_count = 0
        self.last_time = rospy.Time.now()
        #self.PULSES_PER_REV = 60  # 20 slots x 3 gear ratio
        self.WHEEL_RADIUS = 0.035  # in meters
        self.CIRCUMFERENCE = 2 * 3.1416 * self.WHEEL_RADIUS
#         self.DIST_PER_PULSE = self.CIRCUMFERENCE / self.PULSES_PER_REV

        # PID Controller
        self.pid = PIDController(Kp, Ki, Kd, min_throttle, max_throttle)

        # Publisher
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=10)
        self.current_speed_pub = rospy.Publisher('/actual', Float64, queue_size=10)
        # Subscriber to encoder count
        rospy.Subscriber('/arduino_sensor_data', Float64, self.encoder_callback)
        rospy.Subscriber('/desired_speed', Float64, self.desired_speed_callback)
        # User input thread
        
        

        # Control loop
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_callback)

    def encoder_callback(self, msg):
        try:
            rpm  = int(msg.data)
        except ValueError:
            rospy.logwarn("Invalid rpm received from Arduino.")
            return

#         current_time = rospy.Time.now()
#         dt = (current_time - self.last_time).to_sec()
#         if dt <= 0:
#             return

#         delta_pulses = abs(rev_count - self.last_rev_count)
        speed = rpm*2*(3.1416/60)*self.WHEEL_RADIUS

        self.current_speed = speed
#         self.last_rev_count = rev_count
#         self.last_time = current_time

        rospy.loginfo(f"[Encoder] RPM: {rpm}, Speed: {speed:.3f} m/s")
        
    def desired_speed_callback(self , msg):
        self.desired_speed = msg.data
        rospy.loginfo("Received desired speed: %.2f m/s", self.desired_speed)

    def control_callback(self, event):
        error = self.desired_speed - self.current_speed
        throttle_command = self.pid.compute(error)
        rospy.loginfo(f"[Control] Desired: {self.desired_speed:.2f} m/s, Actual: {self.current_speed:.2f} m/s, Throttle: {throttle_command:.2f}")
        self.throttle_pub.publish(throttle_command)
        self.current_speed_pub.publish(self.current_speed)

if __name__ == '__main__':
    try:
        RealCarSpeedController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



