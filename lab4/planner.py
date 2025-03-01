
from mapUtilities import *
from a_star import *
import rclpy

POINT_PLANNER=0; TRAJECTORY_PLANNER=1

class planner:
    def __init__(self, type_, mapName="room"):

        self.type=type_
        self.mapName=mapName


    def plan(self, startPose, endPose):

        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)

        elif self.type==TRAJECTORY_PLANNER:
            self.costMap=None
            self.initTrajectoryPlanner()
            return self.trajectory_planner(startPose, endPose)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):

        self.m_utilites=mapManipulator(laser_sig=0.5)

        self.costMap=self.m_utilites.make_likelihood_field()


    def trajectory_planner(self, startPoseCart, endPoseCart):


        # This is to convert the cartesian coordinates into the
        # the pixel coordinates of the map image, remmember,
        # the cost-map is in pixels. You can by the way, convert the pixels
        # to the cartesian coordinates and work by that index, the a_star finds
        # the path regardless.
        startPose=self.m_utilites.position_2_cell(startPoseCart)
        endPose=self.m_utilites.position_2_cell(endPoseCart)

        pixel_path = search(self.costMap, startPose, endPose, [])
        Path = list(map(self.m_utilites.cell_2_position, pixel_path))

        return Path




if __name__=="__main__":

    rclpy.init()
    m_utilites=mapManipulator()

    map_likelihood=m_utilites.make_likelihood_field()

    path=search(map_likelihood, [0,0], [70,90], [100,90])

    print( list(map(m_utilites.cell_2_position, path)))
