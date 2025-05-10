#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
import tf.transformations as tf_trans

class KalmanFilter:
    def __init__(self, dt):
        self.dt = dt
        self.x = np.zeros((3, 1))  # [x, y, theta]
        self.P = np.eye(3) * 0.1
        self.Q = np.diag([0.01, 0.01, 0.01])
        self.R = np.diag([0.1, 0.1, 0.05])

    def predict(self, u):
        v, omega = u
        theta = self.x[2, 0]

        x_pred = np.array([
            [self.x[0, 0] + v * np.cos(theta) * self.dt],
            [self.x[1, 0] + v * np.sin(theta) * self.dt],
            [self.x[2, 0] + omega * self.dt]
        ])

        F = np.array([
            [1, 0, -v * np.sin(theta) * self.dt],
            [0, 1,  v * np.cos(theta) * self.dt],
            [0, 0, 1]
        ])

        self.x = x_pred
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        H = np.eye(3)
        y = z.reshape(3, 1) - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x += K @ y
        self.P = (np.eye(3) - K @ H) @ self.P

        return self.x.flatten()

class LocalizationNode:
    def __init__(self):

        rospy.init_node('Autonomous_Systems_MS_5_Localization_Team_08')
        self.initialized = False
        self.dt = 0.05  # 20Hz
        self.kf = KalmanFilter(self.dt)

        self.noise_std = {
            'x': 0.05,
            'y': 0.05,
            'theta': 0.002,
            'v': 0.1,
            'omega': 0.05
        }

        self.latest_pose = Pose2D()
        self.v = 0.0
        self.omega = 0.0

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/twist', Twist, self.twist_callback)

        self.filtered_pub = rospy.Publisher('/filtered_pose', Pose2D, queue_size=10)

        rospy.Timer(rospy.Duration(self.dt), self.filter_step)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        orientation_q = [ori.x, ori.y, ori.z, ori.w]
        (_, _, yaw) = tf_trans.euler_from_quaternion(orientation_q)

        if not self.initialized:
            self.kf.x = np.array([[pos.x], [pos.y], [yaw]])
            self.initialized = True
            rospy.loginfo("Kalman Filter Initialized with first /odom reading.")

        # Store the latest ground-truth (even if we're initialized)
        self.latest_pose.x = pos.x
        self.latest_pose.y = pos.y
        self.latest_pose.theta = yaw


    def twist_callback(self, msg):
        self.v = msg.linear.x + np.random.normal(0, self.noise_std['v'])
        self.omega = msg.angular.z + np.random.normal(0, self.noise_std['omega'])

    def filter_step(self, event):
        # Add Gaussian noise to pose measurements
        z = np.array([
            self.latest_pose.x + np.random.normal(0, self.noise_std['x']),
            self.latest_pose.y + np.random.normal(0, self.noise_std['y']),
            self.latest_pose.theta + np.random.normal(0, self.noise_std['theta'])
        ])

        # Kalman filter prediction and update
        self.kf.predict((self.v, self.omega))
        filtered_state = self.kf.update(z)

        # Publish filtered state
        pose = Pose2D()
        pose.x, pose.y, pose.theta = filtered_state
        self.filtered_pub.publish(pose)

if __name__ == '__main__':
    try:
        LocalizationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
