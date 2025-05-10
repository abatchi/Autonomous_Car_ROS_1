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
        self.integral_limit = 1.0  # Conservative anti-windup

    def compute(self, error):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        if dt <= 0:
            return 0.0

        # Proportional term
        P = self.Kp * error
        
        # Conditional integral with clamping
        if abs(error) > 0.05:  # Only integrate when error is significant
            self.integral += error * dt
            self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))
        
        I = self.Ki * self.integral
        
        # Derivative term with smoothing
        D = self.Kd * (error - self.previous_error) / dt if dt > 0 else 0

        output = P + I + D
        output = max(self.min_output, min(output, self.max_output))

        self.previous_error = error
        self.last_time = current_time

        return output

class SpeedController:
    def __init__(self):
        rospy.init_node('Autonomous_Systems_MS_4_CLR_Alg1_Speed_Team_08')
        rospy.loginfo("Node Started...")
        
        # Parameters with guaranteed mobility defaults
        self.desired_speed = rospy.get_param('~desired_speed', 1.0)  # Default to 1.0 m/s
        self.current_speed = 0.0
        self.odom_received = False
        
        # Load other parameters
        Kp = rospy.get_param('~Kp', 2.5)
        Ki = rospy.get_param('~Ki', 0.25)
        Kd = rospy.get_param('~Kd', 0.05)
        min_throttle = rospy.get_param('~min_throttle', 0.0)
        max_throttle = rospy.get_param('~max_throttle', 1.0)
        self.brake_gain = rospy.get_param('~brake_gain', 2)


        # PID controller with aggressive tuning
        # Load parameters for PID controller
        self.pid = PIDController(Kp, Ki, Kd, min_throttle, max_throttle)

        # # Aggressive starting parameters
        # self.pid = PIDController(
        #     Kp= 2.5,   # Strong proportional response
        #     Ki= 0.25,  # Small integral component
        #     Kd= 0.05,  # Minimal derivative
        #     min_output=0.0,
        #     max_output=1.0  # Full throttle capability
        # )
        
        # Brake configuration
        self.brake_gain = 1.5  # Conservative braking
        self.min_move_throttle = 0.4  # Minimum throttle to overcome static friction

        # System state
        self.current_steering = 0.0
        self.last_cmd_time = rospy.Time.now()

        # ROS infrastructure
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)
        
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/steering_cmd', Float64, self.steering_callback)

        # Initialization sequence
        self.initialize_vehicle()
        self.start_control_loop()

    def initialize_vehicle(self):
        """Ensure vehicle starts in known state"""
        self.gear_pub.publish(UInt8(0))  # DRIVE gear
        self.brake_pub.publish(Float64(0.0))
        self.throttle_pub.publish(Float64(0.0))
        rospy.sleep(1)  # Allow time for system initialization

    def start_control_loop(self):
        """Start control thread and timer"""
        self.control_timer = rospy.Timer(rospy.Duration(0.05), self.control_callback)
        if self.desired_speed == 0.0:
            threading.Thread(target=self.keyboard_input_loop, daemon=True).start()

    def steering_callback(self, msg):
        self.current_steering = msg.data

    def odom_callback(self, msg):
        self.current_speed = msg.twist.twist.linear.x
        if not self.odom_received:
            rospy.loginfo(f"Initial speed: {self.current_speed:.2f} m/s")
            self.odom_received = True

    def keyboard_input_loop(self):
        while not rospy.is_shutdown():
            try:
                self.desired_speed = float(input("Enter target speed (m/s): "))
                rospy.loginfo(f"New target: {self.desired_speed:.2f} m/s")
            except ValueError:
                rospy.logwarn("Invalid speed input")

    def control_callback(self, event):
        if not self.odom_received:
            return

        # Calculate control values
        error = self.desired_speed - self.current_speed
        throttle, brake = 0.0, 0.0

        # Mobility-focused control logic
        if abs(self.desired_speed) < 0.05:  # Full stop
            brake = self.full_stop_procedure()
        else:
            throttle, brake = self.motion_control(error)

        # Emergency stop check
        if (rospy.Time.now() - self.last_cmd_time).to_sec() > 1.0:
            rospy.logerr("Control signal timeout! Initiating stop")
            brake = 50.0
            throttle = 0.0

        self.publish_commands(throttle, brake)
        self.last_cmd_time = rospy.Time.now()

    def full_stop_procedure(self):
        """Handle complete stopping"""
        self.gear_pub.publish(UInt8(0))
        return min(abs(self.current_speed) * self.brake_gain, 50.0)

    def motion_control(self, error):
        """Handle moving states"""
        self.handle_gear_selection()
        
        # Static friction override
        if abs(self.current_speed) < 0.1 and abs(self.desired_speed) > 0.2:
            return max(self.pid.compute(error), self.min_move_throttle), 0.0
        
        # Normal operation
        if error > 0.1:  # Accelerate
            return self.pid.compute(error), 0.0
        elif error < -0.1:  # Decelerate
            return 0.0, min(abs(error) * self.brake_gain, 30.0)
        
        return 0.0, 0.0  # Maintain speed

    def handle_gear_selection(self):
        """Select appropriate gear based on desired direction"""
        if self.desired_speed < -0.1:
            self.gear_pub.publish(UInt8(1))  # REVERSE
        else:
            self.gear_pub.publish(UInt8(0))  # DRIVE

    def publish_commands(self, throttle, brake):
        """Publish commands with safety checks"""
        brake = max(0.0, min(brake, 50.0))
        throttle = max(0.0, min(throttle, 1.0))
        
        self.brake_pub.publish(Float64(brake))
        self.throttle_pub.publish(Float64(throttle))
        
        rospy.loginfo(f"Speed: {self.current_speed:.2f}m/s | "
                     f"Throttle: {throttle:.2f} | "
                     f"Brake: {brake:.2f} | "
                     f"Gear: {'DRIVE' if self.desired_speed >=0 else 'REVERSE'}")

if __name__ == '__main__':
    try:
        SpeedController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass