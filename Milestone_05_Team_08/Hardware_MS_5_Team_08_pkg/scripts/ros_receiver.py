#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import String, Float64, Float64MultiArray  # Import Float64 for throttle

def throttle_callback(msg):
    global throttle_command
    throttle_command = msg.data

def servo_callback(msg):
    global servo_command
    servo_command = msg.data

def main():
    global throttle_command
    global servo_command
    throttle_command = 0.0  # Initialize throttle value
    servo_command = 0  # Initialize throttle value
    rospy.init_node('arduino_comm')

    port = rospy.get_param('~port', '/dev/ttyUSB0')
    baudrate = rospy.get_param('~baudrate', 115200)
    sensor_topic = 'arduino_sensor_data'
    sensor_topic_imu = 'imu_data'
    throttle_topic =  'throttle_cmd'
    servo_topic = 'steering_cmd'

    pub = rospy.Publisher(sensor_topic, Float64, queue_size=10)
    pub2 = rospy.Publisher(sensor_topic_imu, Float64, queue_size=10)
    rospy.Subscriber(throttle_topic, Float64, throttle_callback)
    rospy.Subscriber(servo_topic, Float64, servo_callback)
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        rospy.loginfo(f"Connected to Arduino on {port} at {baudrate} baud.")
        rate = rospy.Rate(10)
        rospy.sleep(2)

        while not rospy.is_shutdown():
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                try:
                    parts = line.split(',')
                    if len(parts) == 2:
                        encoder_value = float(parts[0])
                        yaw_value = float(parts[1])
                        rospy.loginfo(f"Published encoder data: {encoder_value},Yaw: {yaw_value}")
                        pub.publish(encoder_value)
                        pub2.publish(-1 * yaw_value)
                    else:
                        rospy.logwarn(f"Unexpected data format: {line}")
                        
                except ValueError:
                    rospy.logwarn(f"Received invalid encoder data: {line}")   
           # Send the latest throttle command to Arduino
            serial_command = f"{int(servo_command)},{throttle_command:.2f}\n"
            ser.write(serial_command.encode())
            
            
            rate.sleep()

    except serial.SerialException as e:
        rospy.logerr(f"Serial Exception: {e}")
    except rospy.ROSInterruptException:
        pass
    finally:
        if ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()

