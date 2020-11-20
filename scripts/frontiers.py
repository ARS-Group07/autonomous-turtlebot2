#!/usr/bin/env python2.7
#
import math

import cv2
import matplotlib.pyplot as plt
import numpy as np
from grids import Grid, GridVisualiser
import rospy
from matplotlib.animation import FuncAnimation
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import CameraInfo, LaserScan
from tf.transformations import euler_from_quaternion


class Pose:

    def __init__(self, px, py, yaw):
        self.px = px
        self.py = py
        self.yaw = yaw

    def update_pose(self, px, py, yaw):
        self.px = px
        self.py = py
        self.yaw = yaw



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



    try:
        rospy.loginfo('fov: ' + str(fov))
        rospy.Subscriber('odom', Odometry, get_odom_data)
        rospy.Subscriber('scan', LaserScan, get_laser_data)



    except rospy.ROSInterruptException:
        rospy.loginfo('ROSInterruptException encountered at %s' % rospy.get_time())
