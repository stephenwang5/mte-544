import sys

from utilities import Logger, euler_from_quaternion
from rclpy.time import Time
from rclpy.node import Node

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from rclpy import init, spin

import rclpy

rawSensor = 0
class localization(Node):
    
    def __init__(self, localizationType=rawSensor):

        super().__init__("localizer")
        
        # Part 3: Define the QoS profile variable based on whether you are using the simulation (Turtlebot 3 Burger) or the real robot (Turtlebot 4)
        # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3

        # TODO: Determine how to differentiate these 
        # #TB4
        odom_qos=QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )
        
        #TB3 Burger
        # odom_qos=QoSProfile(
        #     reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
        #     durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
        #     depth=10,
        # )
        
        self.loc_logger=Logger("robot_pose.csv", ["x", "y", "theta", "stamp"])
        self.pose=None
        
        if localizationType == rawSensor:
        # Part 3: subscribe to the position sensor topic (Odometry)
            self.enc_sub = self.create_subscription(odom, '/odom', self.odom_callback, odom_qos)
        else:
            print("This type doesn't exist", sys.stderr)
    
    def odom_callback(self, pose_msg: odom):
        
        # Part 3: Read x,y, theta, and record the stamp
        self.pose=[ 
            pose_msg.pose.pose.position.x,
            pose_msg.pose.pose.position.y,
            euler_from_quaternion(pose_msg.pose.pose.orientation),
            # pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec * 1e-9
            pose_msg.header.stamp
        ]
        
        # Log the data
        self.loc_logger.log_values([self.pose[0], self.pose[1], self.pose[2], Time.from_msg(self.pose[3]).nanoseconds])
    
    def getPose(self):
        return self.pose

# Part 3
# Here put a guard that makes the node run, ONLY when run as a main thread!
if __name__ == "__main__":
    init()
    LE = localization()
    spin(LE)
