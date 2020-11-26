#!/usr/bin/env python2.7
#
import math

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy
from cv_bridge import CvBridge
from matplotlib.animation import FuncAnimation
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import CameraInfo, LaserScan
from tf.transformations import euler_from_quaternion

from grids import Grid, GridVisualiser
from pose import Pose


class AreaOfInterestFinder:
    def __init__(self, grid):
        self.grid = grid
        cv2.namedWindow('unexplored_contours', 1)

    def get_grid_contours(self):
        image = self.grid.grid
        (h, w) = self.grid.grid.shape
        image = cv2.resize(image, (w * 4, h * 4))  # get larger version for display etc
        image = cv2.flip(image, 0)  # flip mask vertically cuz cv2 :)
        unexplored_mask = cv2.inRange(image, 0.499, 0.501)

        # find all discrete unexplored areas
        _, contours, hierarchy = cv2.findContours(unexplored_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for i, c in enumerate(contours):
            # don't include sub-contours ie. the holes in the white areas
            if hierarchy[0][i][3] != -1:
                continue

            m = cv2.moments(c)
            area = m['m00']

            # don't include very small areas as areas of interest
            if area < 25.:
                continue

            cx = int(m['m10'] / m['m00'])
            cy = int(m['m01'] / m['m00'])

            # draw a circle on each discrete object; size of circle corresponds to area of contour
            cv2.circle(unexplored_mask, (cx, cy), int((area ** 0.33) / 2), 127, -1)

        cv2.imshow('unexplored_contours', unexplored_mask)
        cv2.waitKey(3)


def get_fov(msg):
    """ Loads CameraInfo message and returns horizontal field of view angle. """
    focal_length = msg.K[0]
    w = msg.width

    # formula for getting horizontal field of view angle (rad) from focal length
    fov = 2 * math.atan2(w, (2 * focal_length))

    # return field of view in degrees
    return round(fov * 180 / math.pi)


def create_map_array(map_data, map_meta, grid_resolution):
    """ On startup, get obstacle data from occupancy map and fill in the walls on our grid. """
    map_width = map_meta.width
    map_height = map_meta.height

    grid_size = round(map_width * map_meta.resolution, 1)
    grid_eff_size = int(grid_size / grid_resolution)

    np_map = np.reshape(np.array(map_data), [map_height, map_width]) # convert the 1D input map to a 2D np array
    obstacle_map = np.where(np.logical_or((np_map > 0), (np_map < -0.5)), 255., 0.)  # mask obstacles and unmapped areas
    obstacle_map = cv2.resize(obstacle_map, dsize=(grid_eff_size, grid_eff_size), interpolation=cv2.INTER_AREA)
    obstacle_map = np.where(obstacle_map > 0, -1., 0.5).astype('float32')  # inaccessible areas to -1., all else to 0.5

    return obstacle_map


def get_odom_data(msg, options):
    grid = options['grid']
    pose = options['pose']

    quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                   msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    (_, _, yaw) = euler_from_quaternion(quarternion)
    px = msg.pose.pose.position.x
    py = msg.pose.pose.position.y

    grid.update_grid(px, py, flag='CURR')
    pose.update_pose(px, py, yaw)


def get_laser_data(msg, options):
    density = options['density']
    angles = options['angles']
    grid = options['grid']
    pose = options['pose']
    aoif = options['aoif']

    laser_distances = [msg.ranges[i] for i in angles]

    for angle, dist in zip(angles, laser_distances):
        if math.isinf(dist):  # if laser reads inf distance, clip to the laser's actual max range
            dist = laser_range_max

        plot_points = pose.plot_points_from_laser(angle, dist, density)  # convert to a list of scanned points
        for plot_point in plot_points:
            grid.update_grid(plot_point[0], plot_point[1], flag='NO_OBJ')  # update the grid at each point

    # build contours here
    aoif.get_grid_contours()


if __name__ == '__main__':
    # wait for all important messages to arrive from various nodes
    rospy.init_node('frontier_map', anonymous=True)
    occupancy_map = rospy.wait_for_message('/map', OccupancyGrid, timeout=5).data
    map_metadata = rospy.wait_for_message('/map_metadata', MapMetaData, timeout=5)
    camera_metadata = rospy.wait_for_message('camera/rgb/camera_info', CameraInfo, timeout=5)
    laser_range_max = rospy.wait_for_message('scan', LaserScan, timeout=5).range_max

    # ========== SETTINGS ==========
    pose = Pose(0, 0, 0)
    grid_resolution = 0.2
    laser_density = 4  # eg sample a laser beam every 4 degrees
    fov = get_fov(camera_metadata)  # the field of view of the camera
    rospy.loginfo('fov: ' + str(fov))
    laser_angles = list(range(-int(fov / 2.), 0, laser_density)) + list(range(0, int(fov / 2.), laser_density))

    # grid and visualiser initialisation
    map_arr = create_map_array(occupancy_map, map_metadata, grid_resolution)
    grid = Grid(map_arr=map_arr)
    grid_vis = GridVisualiser(grid)
    aoif = AreaOfInterestFinder(grid)

    odom_options = {'grid': grid, 'pose': pose}
    laser_options = {'density': grid_resolution, 'angles': laser_angles, 'grid': grid, 'pose': pose, 'aoif': aoif}

    try:
        # subscribe to necessary nodes
        rospy.Subscriber('odom', Odometry, get_odom_data, odom_options)
        rospy.Subscriber('scan', LaserScan, get_laser_data, laser_options)

        # begin animation and show the visualiser
        animate = FuncAnimation(grid_vis.fig, grid_vis.plot_grid, init_func=grid_vis.setup_frame)
        plt.show()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo('ROSInterruptException encountered at %s' % rospy.get_time())
