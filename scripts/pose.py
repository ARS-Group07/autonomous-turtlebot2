import math


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
        threshold = 0.5  # to avoid updating on the other side of a wall etc.
        rad_angle = angle * math.pi / 180.
        num_points = int((distance / density) + 1)

        plot_points = []

        for i in range(num_points):
            curr_dist = max(distance - (i * density) - threshold, 0.)
            pxx = self.px + (curr_dist * math.cos(self.yaw + rad_angle))
            pyy = self.py + (curr_dist * math.sin(self.yaw + rad_angle))
            plot_points.append([pxx, pyy])

        return plot_points
