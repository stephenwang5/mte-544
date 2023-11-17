import sys

from utilities import Logger

from rclpy.time import Time

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from sensor_msgs.msg import Imu
from kalman_filter import kalman_filter

from rclpy import init, spin, spin_once

import numpy as np
import message_filters

rawSensors=0
kalmanFilter=1
odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)

class localization(Node):

    def __init__(self, type, dt, loggerName="robotPose.csv", loggerHeaders=["t", "imu_ax", "imu_ay", "odom_omega", "odom_x", "odom_y", "odom_theta", "kf_x", "kf_y", "kf_theta", "kf_omega", "kf_v", "kf_vdot"]):

        super().__init__("localizer")

        self.loc_logger=Logger( loggerName , loggerHeaders)
        self.pose=None

        if type==rawSensors:
            self.initRawSensors()
        elif type==kalmanFilter:
            self.initKalmanfilter(dt)
        else:
            print("We don't have this type for localization", sys.stderr)
            return

    def initRawSensors(self):
        self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)

    def initKalmanfilter(self, dt):

        # TODO Part 3: Set up the quantities for the EKF (hint: you will need the functions for the states and measurements)

        x= np.zeros(6)
        # x = np.array([-0.52, -1.35, -0.8, 0, 0, 0])

        Q= np.eye(6)*0.5

        R= np.eye(4)*0.5

        P = np.eye(6)

        self.kf=kalman_filter(P,Q,R, x, dt)

        # TODO Part 3: Use the odometry and IMU data for the EKF
        self.odom_sub=message_filters.Subscriber(self, odom, "/odom", qos_profile=odom_qos)
        self.imu_sub=message_filters.Subscriber(self, Imu, "/imu", qos_profile=odom_qos)

        time_syncher=message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub], queue_size=10, slop=0.1)
        time_syncher.registerCallback(self.fusion_callback)

        self.first_time = True

    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):
        print(f"called kf at {self.get_clock().now()}")
        odom_theta = euler_from_quaternion(odom_msg.pose.pose.orientation)
        if self.first_time:
            self.kf.x[0] = odom_msg.pose.pose.position.x
            self.kf.x[1] = odom_msg.pose.pose.position.y
            self.kf.x[2] = odom_theta
            self.first_time = False
        print(self.kf.x)

        # TODO Part 3: Use the EKF to perform state estimation
        # Take the measurements
        # your measurements are the linear velocity and angular velocity from odom msg
        # and linear acceleration in x and y from the imu msg
        # the kalman filter should do a proper integration to provide x,y and filter ax,ay
        v = odom_msg.twist.twist.linear.x
        w = odom_msg.twist.twist.angular.z
        ax = imu_msg.linear_acceleration.x
        ay = imu_msg.linear_acceleration.y
        imu_omega = imu_msg.angular_velocity.z
        z = np.array([v, w, ax, ay])
        odom_x = odom_msg.pose.pose.position.x
        odom_y = odom_msg.pose.pose.position.y

        # Implement the two steps for estimation
        self.kf.predict()
        self.kf.update(z)

        # Get the estimate
        xhat = self.kf.get_states()
        x, y, th, _, _, _ = xhat

        t = self.get_clock().now()
        # Update the pose estimate to be returned by getPose
        self.pose=np.array([x, y, th, t.to_msg()])

        # TODO Part 4: log your data
        self.loc_logger.log_values(np.concatenate(
            ([t.nanoseconds * 1e-9, ax, ay, w, odom_x, odom_y, odom_theta], xhat)))

    def odom_callback(self, pose_msg):

        self.pose=[ pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    euler_from_quaternion(pose_msg.pose.pose.orientation),
                    pose_msg.header.stamp]

    # Return the estimated pose
    def getPose(self):
        return self.pose


if __name__=="__main__":

    init()

    LOCALIZER=localization(kalmanFilter, 0.1)

    spin(LOCALIZER)
