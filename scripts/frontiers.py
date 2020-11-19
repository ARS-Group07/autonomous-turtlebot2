#!/usr/bin/env python2.7
#
import math

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy
from matplotlib.animation import FuncAnimation
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import CameraInfo, LaserScan
from tf.transformations import euler_from_quaternion

from grids import Grid, GridVisualiser


class Pose:

    def __init__(self, px, py, yaw):
        self.px = px
        self.py = py
        self.yaw = yaw

    def update_pose(self, px, py, yaw):
        self.px = px
        self.py = py
        self.yaw = yaw

    def plot_points_from_laser(self, angle, distance, density):
        """ Takes in a distance and angle from the laser, and returns a list of points to update on the graph. """
        threshold = 0.2  # to avoid updating on the other side of a wall etc.
        rad_angle = angle * math.pi / 180.
        num_points = int((distance / density) + 1)

        plot_points = []

        for i in range(num_points):
            curr_dist = max(distance - (i * density) - threshold, 0.)
            pxx = self.px + (curr_dist * math.cos(self.yaw + rad_angle))
            pyy = self.py + (curr_dist * math.sin(self.yaw + rad_angle))
            plot_points.append([pxx, pyy])

        return plot_points


def downsample(m, size, ratio):
    """ Resizes an image to size * ratio px. """
    return cv2.resize(m, dsize=(int(size * ratio), int(size * ratio)), interpolation=cv2.INTER_AREA)


def create_map_array(map_data, map_meta):
    """ On startup, get obstacle data from occupancy map and fill in the walls on our grid. """
    map_width = map_meta.width
    map_height = map_meta.height
    map_res = map_meta.resolution

    grid_size = 19.
    grid_res = 0.125
    grid_eff_size = int(grid_size / grid_res)

    np_map = np.reshape(np.array(map_data), [map_width, map_height])  # convert the input map to np array
    np_map = np_map[:380, 4:]  # crop to make our grid fit the map

    obstacle_map = np.where(np_map > 65, 255, 0).astype('float32')  # mask the array where there are obstacles detected
    obstacle_map = cv2.resize(obstacle_map, dsize=(grid_eff_size, grid_eff_size), interpolation=cv2.INTER_AREA)
    obstacle_map = np.where(obstacle_map > 25, -1., -0.25)  # finally assign obstacles to -1., all else to 0.5

    return obstacle_map.ravel()


def get_fov(msg):
    """ Loads CameraInfo message and returns horizontal field of view angle. """
    focal_length = msg.K[0]
    w = msg.width
    rospy.loginfo('Got focal length x: ' + str(focal_length))

    # formula for getting horizontal field of view angle (rad) from focal length
    fov = 2 * math.atan2(w, (2 * focal_length))

    # return field of view in degrees
    return fov * 180 / math.pi


def get_odom_data(msg):
    global grid, pose

    quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                   msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    (_, _, yaw) = euler_from_quaternion(quarternion)
    px = msg.pose.pose.position.x
    py = msg.pose.pose.position.y

    grid.update_grid(px, py, 'CURR')
    pose.update_pose(px, py, yaw)


def get_laser_data(msg):
    global fov, laser_range_max, grid, pose
    point_density = 4  # eg sample a laser beam every 3 degrees
    plot_density = 0.125  # eg plot a probability point every 0.125 metres

    laser_range = int(fov / 2.)  # the positive and negative fov angles
    laser_angles = list(range(-laser_range, 0, point_density)) + list(range(0, laser_range, point_density))
    laser_distances = [msg.ranges[i] for i in laser_angles]  # sample the laser data every (point_density) points

    # iterate through all those laser readings
    for angle, dist in zip(laser_angles, laser_distances):
        if math.isinf(dist):  # if laser reads inf distance, clip to the laser's actual max range
            dist = laser_range_max

        plot_points = pose.plot_points_from_laser(angle, dist, plot_density)  # convert to a list of scanned points
        for plot_point in plot_points:
            grid.update_grid(plot_point[0], plot_point[1], 'NO_OBJ')  # update the grid at each point


if __name__ == '__main__':

    rospy.init_node('frontier_map', anonymous=True)
    pose = Pose(0, 0, 0)

    # Initialise sensors and important attributes
    occupancy_map = rospy.wait_for_message('/map', OccupancyGrid, timeout=5).data
    map_metadata = rospy.wait_for_message('/map_metadata', MapMetaData, timeout=5)
    rospy.loginfo('occupancy map type: ' + str(type(occupancy_map)))
    rospy.loginfo('occupancy map shape: ' + str(len(occupancy_map)))
    rospy.loginfo('occupancy map metadata: \nheight: ' + str(map_metadata.height) +
                  '\nwidth: ' + str(map_metadata.width) +
                  '\nresolution: ' + str(map_metadata.resolution))

    camera_metadata = rospy.wait_for_message('camera/rgb/camera_info', CameraInfo, timeout=5)
    laser_range_max = rospy.wait_for_message('scan', LaserScan, timeout=5).range_max  # max distance that laser detects
    fov = get_fov(camera_metadata)  # the field of view of the camera

    rospy.loginfo('laser range: ' + str(laser_range_max))

    map_arr = create_map_array(occupancy_map, map_metadata)

    grid = Grid(map_arr=map_arr)
    grid_vis = GridVisualiser(grid)

    try:
        rospy.loginfo('fov: ' + str(fov))
        rospy.Subscriber('odom', Odometry, get_odom_data)
        rospy.Subscriber('scan', LaserScan, get_laser_data)

        animate = FuncAnimation(grid_vis.fig, grid_vis.plot_grid, init_func=grid_vis.setup_frame)
        plt.show()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo('ROSInterruptException encountered at %s' % rospy.get_time())
