#!/usr/bin/env python3

import rospy
import threading
from std_msgs.msg import Float64, UInt8
from nav_msgs.msg import Odometry

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

class SpeedController:
    def __init__(self):
        rospy.init_node('Autonomous_Systems_MS_3_CLR_Alg1_Speed_Team_08')
        rospy.loginfo("Node Started...")

        self.desired_speed = 0.0
        self.current_speed = None
        self.odom_received = False

        # Load parameters
        Kp = rospy.get_param('~Kp', 0.1)
        Ki = rospy.get_param('~Ki', 0.0)
        Kd = rospy.get_param('~Kd', 0.01)
        min_throttle = rospy.get_param('~min_throttle', 0.0)
        max_throttle = rospy.get_param('~max_throttle', 1.0)
        self.brake_gain = rospy.get_param('~brake_gain', 5)

        self.pid = PIDController(Kp, Ki, Kd, min_throttle, max_throttle)

        # Publishers
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=10)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=10)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=10)

        # Initialize vehicle
        rospy.sleep(1)
        self.gear_pub.publish(UInt8(0))  # Default to DRIVE
        self.brake_pub.publish(Float64(0.0))
        rospy.sleep(1)

        # Subscribers and timers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_callback)

        # Keyboard input thread
        self.input_thread = threading.Thread(target=self.keyboard_input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def odom_callback(self, msg):
        self.current_speed = msg.twist.twist.linear.x
        if not self.odom_received:
            self.odom_received = True
            rospy.loginfo(f"[Odom] First speed received: {self.current_speed:.2f} m/s")

    def keyboard_input_loop(self):
        while not rospy.is_shutdown():
            try:
                user_input = input("Enter desired speed (m/s): ")
                self.desired_speed = float(user_input)
                rospy.loginfo(f"[User Input] Desired speed updated to: {self.desired_speed:.2f} m/s")
            except ValueError:
                rospy.logwarn("Invalid input. Please enter a valid float.")

    def control_callback(self, event):
        if not self.odom_received:
            rospy.logwarn("[Speed Ctrl] Waiting for odometry data...")
            return

        error = self.desired_speed - self.current_speed
        throttle = 0.0
        brake = 0.0

        # Handle zero desired speed (full brake)
        if abs(self.desired_speed) < 0.01:
            brake = min(abs(self.current_speed) * self.brake_gain, 50)
            self.gear_pub.publish(UInt8(0))  # Default gear when stopping
        else:
            # Set gear based on desired direction
            if self.desired_speed < 0:
                self.gear_pub.publish(UInt8(1))  # REVERSE
            else:
                self.gear_pub.publish(UInt8(0))  # DRIVE

            # Compute throttle and brake
            
            if error < -0.1:  # Brake if overspeeding
                if (self.current_speed <= 0):
                    throttle = self.pid.compute(abs(error))
                    brake = 0.0
                
                else:
                    brake = min(abs(error) * self.brake_gain, 50)
                    throttle = 0.0

            if error > 0.1:  # Brake if overspeeding
                if (self.current_speed >= 0):
                    throttle = self.pid.compute(error)
                    brake = 0.0
                
                else:
                    brake = min(abs(error) * self.brake_gain, 50)
                    throttle = 0.0

        # Publish commands
        self.brake_pub.publish(Float64(brake))
        self.throttle_pub.publish(Float64(throttle))

        rospy.loginfo(f"[Speed Ctrl] Desired: {self.desired_speed:.2f} | Current: {self.current_speed:.2f} | Throttle: {throttle:.2f} | Brake: {brake:.2f}")

if __name__ == '__main__':
    try:
        SpeedController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass