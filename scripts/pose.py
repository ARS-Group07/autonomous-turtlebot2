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

    def plot_points_from_laser(self, angle, distance, density):
        """ Takes in a distance and angle from the laser, and returns a list of points to update on the graph. """
        threshold = 0.2  # to avoid updating on the other side of a wall etc.
        rad_angle = angle * math.pi / 180.
        num_points = int((distance / density) + 1)

        plot_points = []

        for i in range(num_points):
            curr_dist = max(distance - (i * density) - threshold, 0.)
            pxx = self.x + (curr_dist * math.cos(self.yaw + rad_angle))
            pyy = self.y + (curr_dist * math.sin(self.yaw + rad_angle))
            plot_points.append([pxx, pyy])

        return plot_points