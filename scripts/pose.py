import math
import numpy as np
import random

class Pose:
    """ Class representing pose information. Functions for calculating linear, angular distance between poses. """
    def __init__(self, x=0., y=0., yaw=0.):
        self.x = x
        self.y = y
        self.yaw = yaw

    def lin_dist(self, other_pose):
        """ Get the distance travelled (L2 vector norm of x, y). """
        dx = self.x - other_pose.x
        dy = self.y - other_pose.y

        return math.sqrt(dx ** 2 + dy ** 2)

    def ang_dist(self, other_pose):
        # get angle vectors
        other_front = (math.cos(other_pose.yaw), math.sin(other_pose.yaw))
        front = (math.cos(self.yaw), math.sin(self.yaw))

        # assign as unit vectors
        other_front = other_front / np.linalg.norm(other_front)
        front = front / np.linalg.norm(front)

        # return angle difference
        return np.arccos(np.dot(front, other_front))

    def get_random_yaw(self):
        # get random yaw to turn towards
        ang_dist = 0.
        yaw = 0.

        while ang_dist < math.pi / 4:
            yaw = math.radians(random.randrange(-180, 180))
            ang_dist = self.ang_dist(Pose(0., 0., yaw))

        return yaw