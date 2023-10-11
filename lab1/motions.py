# Imports
import rclpy

from rclpy.node import Node

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile
import rclpy.qos as rqos

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time

CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        super().__init__("motion_types")
        
        self.type=motion_type
        
        self.radius_=0.0
        
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False
        
        # Part 3: publisher to send velocity commands by setting the proper parameters
        self.vel_publisher=self.create_publisher(Twist, '/cmd_vel', 10)
                
        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "stamp"])
        
        # Part 3: QoS profile by setting the proper parameters - verified that the reliability/durability is correct
        qos=QoSProfile(
            reliability=rqos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rqos.QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        # Part 5: subscription to the topics corresponding to the respective sensors
        
        # IMU subscription
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, qos)
        # ENCODER subscription
        self.enc_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos)
        # LaserScan subscription 
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        # Initial value of the robot
        self.speed=0.0
        self.create_timer(0.1, self.timer_callback)


    # Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py

    def imu_callback(self, imu_msg: Imu):
        self.imu_initialized = True
        self.imu_logger.log_values([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.angular_velocity.z,
            imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9,
        ]) # log imu msgs
        
    def odom_callback(self, odom_msg: Odometry):
        self.odom_initialized = True
        self.odom_logger.log_values([
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            euler_from_quaternion(odom_msg.pose.pose.orientation),
            odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9,
        ]) # log odom msgs
                
    def laser_callback(self, laser_msg: LaserScan):
        self.laser_initialized = True
        self.laser_logger.log_values([
            laser_msg.ranges,
            laser_msg.header.stamp.sec + laser_msg.header.stamp.nanosec * 1e-9,
        ]) # log laser msgs with position msg at that time
                
    def timer_callback(self):
        
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
            
        if not self.successful_init:
            return
        
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        self.vel_publisher.publish(cmd_vel_msg)
        
    
    # Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot

    def make_circular_twist(self):
        msg=Twist()

        # constant linear veloity + constant angular velocity = circular motion
        msg.linear.x = 1.
        msg.angular.z = 1.

        return msg

    def make_spiral_twist(self):
        msg=Twist()

        # increase linear speed at each timer callback (0.1 seconds)
        self.speed+=0.005
        msg.linear.x = self.speed
        # keep angular velocity constant with increasing linear velocity to create spiral motion
        msg.angular.z = 1.0

        return msg
    
    def make_acc_line_twist(self):
        msg=Twist()

        # constant linear velocity with 0 angular velocity creates straight line motion
        # from instructions, it was assumed that this straight line did not need to accelerate as there was no specification
        msg.linear.x = 1.

        return msg

import argparse

if __name__=="__main__":
    

    argParser=argparse.ArgumentParser(description="input the motion type")


    argParser.add_argument("--motion", type=str, default="circle")



    rclpy.init()

    args = argParser.parse_args()

    if args.motion.lower() == "circle":

        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)

    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)

    else:
        print(f"we don't have {arg.motion.lower()} motion type")


    
    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
