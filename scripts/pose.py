import math
import numpy as np
import random
from scipy.spatial.transform import Rotation as R


class Pose:
    """ Class representing pose information. Functions for calculating linear, angular distance between poses. """

    def __init__(self, x=0., y=0., yaw=0.):
        self.px = x
        self.py = y
        self.yaw = yaw

    def update_pose(self, px, py, yaw):
        self.px = px
        self.py = py
        self.yaw = yaw

    def plot_points_from_laser(self, angle, distance, density):
        """ Takes in a distance and angle from the laser, and returns a list of points to update on the graph. """
        threshold = 1.  # to avoid updating on the other side of a wall etc.
        rad_angle = angle * math.pi / 180.
        num_points = int((distance / density) + 1)

        plot_points = []

        for i in range(num_points):
            curr_dist = max(distance - (i * density) - threshold, 0.)
            pxx = self.px + (curr_dist * math.cos(self.yaw + rad_angle))
            pyy = self.py + (curr_dist * math.sin(self.yaw + rad_angle))
            plot_points.append([pxx, pyy])

        return plot_points

    def ang_dist(self, other_pose):
        # get angle vectors
        other_front = (math.cos(other_pose.yaw), math.sin(other_pose.yaw))
        front = (math.cos(self.yaw), math.sin(self.yaw))

        # assign as unit vectors
        other_front = other_front / np.linalg.norm(other_front)
        front = front / np.linalg.norm(front)

        # return angle difference
        return np.arccos(np.dot(front, other_front))

    def dist(self, other_pose):
        return math.sqrt((self.px - other_pose.px) ** 2 + (self.py - other_pose.py) ** 2)

    def get_random_yaw(self):
        # get random yaw to turn towards
        ang_dist = 0.
        yaw = 0.

        # choose an angle at least 45 degrees from current position
        while ang_dist < math.pi / 4:
            yaw = math.radians(random.randrange(-180, 180))
            ang_dist = self.ang_dist(Pose(0., 0., yaw))

        return yaw

    def locate(self, pose2):
        if pose2.px > 0:
            pose2.yaw = -pose2.yaw
        vec = [pose2.px, pose2.py, 0]
        newyaw = self.yaw + pose2.yaw
        rotation_radians = self.yaw
        rotation_axis = np.array([0, 0, 1])
        rotation_vector = rotation_radians * rotation_axis
        rotation = R.from_rotvec(rotation_vector)
        rotated_vec = rotation.apply(vec)

        x = self.px + rotated_vec[0]
        y = self.py + rotated_vec[1]

        return Pose(x, y, newyaw)
