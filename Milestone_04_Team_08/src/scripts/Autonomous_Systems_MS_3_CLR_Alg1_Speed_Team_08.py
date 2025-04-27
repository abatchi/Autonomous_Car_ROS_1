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
        self.prev_error = 0.0
        self.last_time = rospy.Time.now()
        self.integral_limit = 0.8  # Tuned anti-windup

    def compute(self, error):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        if dt <= 0:
            return 0.0

        # Conditional integration
        if abs(error) > 0.02:  # Smaller error threshold
            self.integral += error * dt
            self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))
        
        # PID components
        P = self.Kp * error
        I = self.Ki * self.integral
        D = self.Kd * (error - self.prev_error) / dt if dt > 0 else 0

        output = P + I + D
        output = max(self.min_output, min(output, self.max_output))

        self.prev_error = error
        self.last_time = current_time
        return output

class SpeedController:
    def __init__(self):
        rospy.init_node('Autonomous_Systems_MS_4_CLR_Alg1_Speed_Team_08')
        
        # Planning node integration
        self.desired_speed = 0.0
        self.speed_sub = rospy.Subscriber('/desired_speed', Float64, self.speed_callback)
        
        # Load other parameters
        Kp = rospy.get_param('~Kp', 0.1)
        Ki = rospy.get_param('~Ki', 0.0)
        Kd = rospy.get_param('~Kd', 0.01)
        min_throttle = rospy.get_param('~min_throttle', 0.0)
        max_throttle = rospy.get_param('~max_throttle', 1.0)
        self.brake_gain = rospy.get_param('~brake_gain', 5)

        self.pid = PIDController(Kp, Ki, Kd, min_throttle, max_throttle)
        
        # Braking parameters
        self.brake_gain = 1.2  # Reduced from 2.0
        self.min_move_throttle = 0.25  # Lower minimum throttle
        
        # System state
        self.current_speed = 0.0
        self.odom_received = False
        self.last_cmd_time = rospy.Time.now()
        self.gear = 0  # 0 for DRIVE, 1 for REVERSE, initial gear is DRIVE

        # ROS interfaces
        self.throttle_pub = rospy.Publisher('/throttle_cmd', Float64, queue_size=1)
        self.gear_pub = rospy.Publisher('/gear_cmd', UInt8, queue_size=1)
        self.brake_pub = rospy.Publisher('/brake_cmd', Float64, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.initialize_vehicle()
        rospy.Timer(rospy.Duration(0.05), self.control_callback)

    def initialize_vehicle(self):
        """Initialize vehicle to safe state"""
        self.gear_pub.publish(UInt8(0))  # DRIVE
        self.brake_pub.publish(Float64(0.0))
        self.throttle_pub.publish(Float64(0.0))
        rospy.sleep(0.5)

    def speed_callback(self, msg):
        """Handle speed commands from planning node"""
        self.desired_speed = msg.data
        rospy.loginfo_throttle(1, f"New target speed: {self.desired_speed:.2f} m/s")

    def odom_callback(self, msg):
        """Update current vehicle speed"""
        self.current_speed = msg.twist.twist.linear.x
        if not self.odom_received:
            rospy.loginfo(f"Initial speed: {self.current_speed:.2f} m/s")
            self.odom_received = True

    def control_callback(self, event):
        """Main control loop"""
        if not self.odom_received:
            return

        error = self.desired_speed - self.current_speed
        throttle, brake = 0.0, 0.0

        # Handle different control regimes
        if abs(self.desired_speed) < 0.05:
            brake = self.full_stop_procedure()
        else:
            throttle, brake = self.speed_regulation(error)

        # Emergency system
        if (rospy.Time.now() - self.last_cmd_time).to_sec() > 2.0:
            rospy.logerr("Control timeout! Emergency stop")
            brake = 50.0
            throttle = 0.0

        self.publish_commands(throttle, brake)
        self.last_cmd_time = rospy.Time.now()

    def speed_regulation(self, error):
        """Calculate throttle and brake commands"""
        self.handle_gear_selection()
        
        # Static friction compensation
        if abs(self.current_speed) < 0.15 and abs(self.desired_speed) > 0.2:
            return max(self.pid.compute(error), self.min_move_throttle), 0.0
        
        # Speed control logic
        if error > 0.05:  # Accelerate
            return self.pid.compute(error), 0.0
        elif error < -0.05:  # Decelerate
            return 0.0, min(abs(error) * self.brake_gain, 25.0)
        
        return 0.0, 0.0  # Maintain speed

    def handle_gear_selection(self):
        """Smart gear selection with hysteresis"""
        if self.desired_speed < -0.5 or (self.desired_speed < -0.1 and self.current_speed < -0.2):
            if self.gear != 1:
                self.gear = 1
                self.gear_pub.publish(UInt8(1))  # REVERSE
        elif self.desired_speed > 0.1 or (self.desired_speed > -0.1 and self.current_speed > 0.2):
            if self.gear != 0:
                self.gear = 0
                self.gear_pub.publish(UInt8(0))  # DRIVE

    def full_stop_procedure(self):
        """Smooth stopping procedure"""
        return min(abs(self.current_speed) * self.brake_gain * 1.5, 40.0)

    def publish_commands(self, throttle, brake):
        """Publish final commands with safety checks"""
        brake = max(0.0, min(brake, 50.0))
        throttle = max(0.0, min(throttle, 0.8))
        
        self.brake_pub.publish(Float64(brake))
        self.throttle_pub.publish(Float64(throttle))
        
        rospy.loginfo_throttle(0.5,
            f"Speed: {self.current_speed:.2f} m/s | "
            f"Throttle: {throttle:.2f} | "
            f"Brake: {brake:.2f} | "
            f"Gear: {'DRIVE' if self.gear == 0 else 'REVERSE'}"
        )

if __name__ == '__main__':
    try:
        SpeedController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
