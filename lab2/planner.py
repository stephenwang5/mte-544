import numpy as np
import matplotlib.pyplot as plt

# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1

class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0, 0.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.generate_sigmoid_trajectory()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        theta = goalPoint[2]
        return x, y, theta

    # TODO Part 6: Implement the trajectories here
    def generate_parabola_trajectory(self):
        points = 21
        x = np.arange(points) / ((points-1)/2) -1
        # x = np.arange(points)/points * 2 - 0.9
        y = x**2
        plt.plot(x, y)
        plt.show()
        with open("parabola.npz", "wb") as f:
            np.savez(f, x, y)
        ret = np.concatenate(
            (x.reshape(-1,1), y.reshape(-1,1)),
            axis=1
        )
        return ret.tolist()
        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        # return 

    def generate_sigmoid_trajectory(self):
        points = 21
        x = np.arange(points) / points * 4 - 2
        y = 1 / (1 + np.exp(-5*x))
        with open("sigmoid.npz", "wb") as f:
            np.savez(f, x, y)
        plt.plot(x, y)
        plt.show()

        ret = np.concatenate(
            (x.reshape(-1,1), y.reshape(-1,1)),
            axis=1
        )
        return ret.tolist()
