# Imports


import sys

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensor

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController

# You may add any other imports you may need/want to use below
import rclpy

THRESHOLD = 0.1
THRESHOLD_LIN = THRESHOLD
THRESHOLD_ANG = THRESHOLD

# goalPoint = [-1.0, -1.0, 0.0]
goalPoint = [-2.5, 0, 0.0]

class decision_maker(Node):
    
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, goalPoint, rate=10, motion_type=POINT_PLANNER):

        super().__init__("decision_maker")

        #Part 4: Create a publisher for the topic responsible for robot's motion
        self.publisher=self.create_publisher(publisher_msg, publishing_topic, qos_publisher)

        publishing_period=1/rate
        
        # Instantiate the controller
        # TODO Part 5: Tune your parameters here
    
        if motion_type == POINT_PLANNER:
            self.controller=controller(klp = 1.0,
                                       klv = 0.1,
                                       kli = 0.1,
                                    #    klv = 1.25*0.8, 
                                    #    kli = 0.25*0.8,

                                       kap = 1.,
                                       kav = 0.3,
                                       kai = 0.1,
                                    )
            self.planner=planner(POINT_PLANNER)
            self.goal=self.planner.plan(goalPoint)
    
    
        elif motion_type==TRAJECTORY_PLANNER:
            self.controller=trajectoryController(klp=2.5, 
                                                 klv=1.0, 
                                                 kli=0.05,

                                                 kap=2.0, 
                                                 kav=0.5, 
                                                 kai=0.01
                                                )

            # self.controller=trajectoryController(klp=0.2, 
            #                             klv=0.2, 
            #                             kli=0.2,

            #                             kap=0.2, 
            #                             kav=0.2, 
            #                             kai=0.2
            #                         )
            self.planner=planner(TRAJECTORY_PLANNER)
            self.goal=self.planner.plan()

        else:
            print("Error! you don't have this planner", file=sys.stderr)


        # Instantiate the localization, use rawSensor for now  
        self.localizer=localization(rawSensor)

        # Instantiate the planner
        # NOTE: goalPoint is used only for the pointPlanner
        # self.goal=self.planner.plan(goalPoint)

        self.create_timer(publishing_period, self.timerCallback)


    def timerCallback(self):
        
        # Part 3: Run the localization node
        # Remember that this file is already running the decision_maker node.
        # i think we need to call spin_once here and give it a node. Ref to 9:45 of lab2 tut
        spin_once(self.localizer)

        if self.localizer.getPose()  is  None:
            print("waiting for odom msgs ....")
            return

        vel_msg=Twist()
        
        # Part 3: Check if you reached the goal
        # Check if we reached the goal:
        #   if list then we need to see if our trajectory is correct
        #   if not list, then are we at the correct point?        
        if type(self.goal) == list:
            e_lin = calculate_linear_error(self.localizer.getPose(), self.goal[-1])
            a_lin = calculate_angular_error(self.localizer.getPose(), self.goal[-1])            
            print(f"e_lin: {e_lin}\na_lin: {a_lin}")
            print(f"pose: {self.localizer.getPose()}")
            print(f"goal: {self.goal}")
            reached_goal = (e_lin < THRESHOLD_LIN)
        else: 
            e_lin = calculate_linear_error(self.localizer.getPose(), self.goal)
            a_lin = calculate_angular_error(self.localizer.getPose(), self.goal)
            print(f"e_lin: {e_lin}\na_lin: {a_lin}")
            print(f"pose: {self.localizer.getPose()}")
            print(f"goal: {self.goal}")
            # reached_goal = ((e_lin < THRESHOLD_LIN) and (a_lin < THRESHOLD_ANG))
            reached_goal = (e_lin < THRESHOLD_LIN)

        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg)
            
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()
            
            # Part 3: exit the spin
            raise SystemExit
        
        velocity, yaw_rate = self.controller.vel_request(self.localizer.getPose(), self.goal, True)

        # Part 4: Publish the velocity to move the robot
        vel_msg.linear.x = velocity
        vel_msg.angular.z = yaw_rate
        self.publisher.publish(vel_msg)

import argparse


def main(args=None):
    
    init()

    # Part 3: You migh need to change the QoS profile based on whether you're using the real robot or in simulation.
    # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
    
    # TODO: Determine how to differentiate these 
    # #TB4
    # odom_qos=QoSProfile(
    #     reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
    #     durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
    #     depth=10,
    # )
    
    #TB3 Burger
    odom_qos=QoSProfile(
        reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
        durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
        depth=10,
    )
    # odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)
    

    # Part 3: instantiate the decision_maker with the proper parameters for moving the robot
    if args.motion.lower() == "point":
        DM=decision_maker(
            publisher_msg=Twist, 
            publishing_topic='/cmd_vel', 
            qos_publisher=odom_qos, 
            goalPoint=goalPoint, # TODO: Is this hardcoded 
            rate=10,
            motion_type= POINT_PLANNER
        )
    elif args.motion.lower() == "trajectory":
        DM=decision_maker(
            publisher_msg=Twist, 
            publishing_topic='/cmd_vel', 
            qos_publisher=odom_qos, 
            goalPoint=goalPoint, # TODO: Is this hardcoded 
            rate=10,
            motion_type= TRAJECTORY_PLANNER
        )
    else:
        print("invalid motion type", file=sys.stderr)        
    
    
    
    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")


if __name__=="__main__":

    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="point")
    args = argParser.parse_args()

    main(args)
