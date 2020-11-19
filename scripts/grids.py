import numpy as np
import rospy
from matplotlib import pyplot as plt


class Grid:
    """ Class representing a metric map, denoting obstacles and probabilities of objects of interest in the world. """

    def __init__(self, size=19., resolution=0.125, map_arr=None):
        self.origin_x = -10.  # from map .yaml file
        self.origin_y = -10.
        self.size = size
        self.resolution = resolution
        self.eff_size = int(self.size / self.resolution)  # effective size used in most calculations
        self.prev_index = None

        # Create a 1D array of 0.5's that will represent the probability grid
        # Initialise grid from the map
        self.grid = map_arr
        # self.grid = np.ones([self.eff_size ** 2]) * -0.25

    def update_grid(self, px, py, flag):
        """ Based on camera data, update the probability of an object of interest being present at each co-ordinate. """
        gx, gy = self.to_grid(px, py)

        if self.grid[self.to_index(gx, gy)] == -1.:
            return

        if flag == 'CURR':
            if self.prev_index:
                self.grid[self.prev_index] = 0.5

            self.prev_index = self.to_index(gx, gy)
            self.grid[self.to_index(gx, gy)] = 2.

        elif flag == 'NO_OBJ':
            self.grid[self.to_index(gx, gy)] = 0.5

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

    def to_index(self, gx, gy):
        """ Get the index of the point (gx, gy) in the 1D vector form of the grid. """
        return int(gy * self.eff_size + gx)


class GridVisualiser:
    """ Visualiser class for the grid. """

    def __init__(self, input_grid):
        self.grid = input_grid
        self.fig, self.ax = plt.subplots()
        self.fig.set_size_inches(6, 6)
        self.cmap = plt.get_cmap('coolwarm')

        plt.xlabel('gx')
        plt.ylabel('gy')
        # plt.gca().invert_xaxis()
        plt.title('Object of interest metric map (absolute)')

    def setup_frame(self):
        self.ax.set_xlim(0, self.grid.eff_size)
        self.ax.set_ylim(0, self.grid.eff_size)

    def plot_grid(self, frame):
        grid_2d = np.reshape(self.grid.grid, (self.grid.eff_size, self.grid.eff_size))
        self.ax.pcolormesh(grid_2d, cmap=self.cmap, vmin=-1., vmax=2.)
