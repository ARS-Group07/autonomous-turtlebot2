import rospy
import cv2
import numpy as np

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

        self.shape_x = input_grid.grid.shape[0]
        self.shape_y = input_grid.grid.shape[1]

        print ("Shape X: " + str(self.shape_x))
        print ("Shape Y: " + str(self.shape_x))

        cv2.namedWindow('grid_vis', 2)
        self.update_plot()

    def update_plot(self):
        image = np.array(self.grid.grid)
        image = np.where(image==0, 0.15, image)
        #image = np.where(image==255, 0.25, image)
        #image = np.where(image==0.5, 0.5, image)
        #image = np.where(image==2, 0.75, image)
        #image = np.where(image==-1, 1, image) # Replace all -1 values with 1

        image = cv2.resize(image, (self.shape_x * 4, self.shape_y * 4))  # get larger version for display etc
        image = cv2.flip(image, 0)  # flip mask vertically cuz cv2 :)
        cv2.imshow('grid_vis', image)
        cv2.waitKey(1)

