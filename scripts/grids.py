import matplotlib.colors
import numpy as np
import rospy
from matplotlib import pyplot as plt


class Grid:
    """ Class representing a metric map, denoting obstacles and probabilities of objects of interest in the world. """

    def __init__(self, size=19.2, resolution=0.2, map_arr=None):
        self.origin_x = -10.  # from map .yaml file
        self.origin_y = -10.
        self.size = size
        self.resolution = resolution
        self.eff_size = int(self.size / self.resolution)  # effective size used in most calculations
        self.prev_index = ()

        # Create a 2D array of 0.5's using the map, which will represent the probability grid
        self.grid = map_arr
        rospy.loginfo('self.grid shape: ' + str(self.grid.shape))

    def update_grid(self, px, py, flag):
        """ Based on camera data, update the probability of an object of interest being present at each co-ordinate. """
        gx, gy = self.to_grid(px, py)

        # don't update the grid squares in the walls
        if self.grid[gy, gx] == -1. and flag == 'NO_OBJ':
            return

        if flag == 'CURR':
            # for updating the position of the robot on the map
            if self.prev_index:
                self.grid[self.prev_index[0], self.prev_index[1]] = 0.

            self.prev_index = (gy, gx)
            self.grid[gy, gx] = 2.

        elif flag == 'NO_OBJ':
            # for marking the areas the robot has seen
            if (gy, gx) == self.prev_index:
                return
            self.grid[gy, gx] = 0.

    def to_grid(self, px, py):
        """ Given an odometry point (px, py), return the grid point (gx, gy). """
        gx = (px - self.origin_x) / self.resolution
        gy = (py - self.origin_y) / self.resolution
        return int(gx), int(gy)

    def to_world(self, gx, gy):
        """ Given a grid point (gx, gy), return the odometry point (px, py). """
        px = gx * self.resolution + self.origin_x
        py = gy * self.resolution + self.origin_y
        return int(px), int(py)


class GridVisualiser:
    """ Visualiser class for the grid. """

    def __init__(self, input_grid):
        self.grid = input_grid
        self.fig, self.ax = plt.subplots()
        self.fig.set_size_inches(6, 6)
        self.cmap = matplotlib.colors.LinearSegmentedColormap.from_list("",
                    ["darkslategrey", "darkslategrey", "white", "silver", "tan", "plum", "coral", "lime", "red"])

        plt.xlabel('gx')
        plt.ylabel('gy')
        plt.title('Exploration metric map (absolute)')

    def setup_frame(self):
        self.ax.set_xlim(0, self.grid.eff_size)
        self.ax.set_ylim(0, self.grid.eff_size)

    def plot_grid(self, frame):
        self.ax.pcolormesh(self.grid.grid, cmap=self.cmap, vmin=-1., vmax=3.)
