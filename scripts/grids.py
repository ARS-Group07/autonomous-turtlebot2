import rospy
import cv2
import numpy as np


class Grid:

    def __init__(self, size=19.2, resolution=0.2, map_arr=None):
        self.origin_x = -10.  # from map .yaml file
        self.origin_y = -10.
        self.size = size
        self.resolution = resolution
        self.eff_size = int(self.size / self.resolution)  # effective size used in most calculations
        self.prev_index = ()

        # Create a 2D array of 0.5's using the map, which will represent the probability grid
        self.grid = np.copy(map_arr)
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
        return px, py

    def is_fully_explored(self):
        unexplored_mask = np.where(self.grid == 0.5, 1, 0)
        unexplored_points = np.sum(unexplored_mask)
        return unexplored_points < 230

    def reset_grid(self, map_arr):
        self.grid = np.copy(map_arr)


class GridVisualiser:
    """ Visualiser class for the grid. """

    def __init__(self, input_grid):
        self.grid = input_grid
        self.lut = self.generate_lut()

        self.shape_x = input_grid.grid.shape[0]
        self.shape_y = input_grid.grid.shape[1]

        print ("Shape X: " + str(self.shape_x))
        print ("Shape Y: " + str(self.shape_x))

        cv2.namedWindow('Map of explored space', 2)
        self.update_plot()

    @staticmethod
    def generate_lut():
        """ Set colour map for colouring in the visualiser. """
        lut = np.zeros([256, 3], dtype=np.uint8)

        # order is obstacles, explored space, unexplored space, robot
        # set blues
        lut[:64, 0] = 135
        lut[64:92, 0] = 252
        lut[92:192, 0] = 200
        lut[192:, 0] = 69

        # set greens
        lut[:64, 1] = 129
        lut[64:92, 1] = 253
        lut[92:192, 1] = 200
        lut[192:, 1] = 146

        # set reds
        lut[:64, 2] = 58
        lut[64:92, 2] = 255
        lut[92:192, 2] = 200
        lut[192:, 2] = 255

        return lut

    def update_plot(self):
        image = ((self.grid.grid + 1.) * 64).astype(np.uint8)  # convert image to 0,255 range
        image = cv2.resize(image, (self.shape_x * 4, self.shape_y * 4), interpolation=cv2.INTER_NEAREST)  # resize for display
        image = cv2.flip(image, 0)  # flip mask vertically cuz cv2 :)

        colour_image = np.empty([self.shape_x * 4, self.shape_y * 4, 3], dtype=np.uint8)
        for i in range(3):
            colour_image[..., i] = cv2.LUT(image, self.lut[:, i])

        cv2.imshow('Map of explored space', colour_image)
        cv2.waitKey(1)
