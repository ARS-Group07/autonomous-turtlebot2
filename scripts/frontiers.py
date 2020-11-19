#!/usr/bin/env python2.7
#
import math

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy
from matplotlib.animation import FuncAnimation
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, LaserScan

from grids import Grid, GridVisualiser


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
    global grid

    px = msg.pose.pose.position.x
    py = msg.pose.pose.position.y
    # rospy.loginfo('GRID: px py co-ordinates: (' + str(px) + ', ' + str(py) + ')')

    grid.update_grid(px, py)


def get_laser_data(msg):
    global fov, laser_range_max

    # get the laser data for the camera's field of view
    laser_range = int(fov / 2.)
    laser_data = msg.ranges[:laser_range] + msg.ranges[-laser_range:]

    # todo list

    # todo 1.
    # need to somehow find all the grid squares up to the points detected by the laser
    # so lets say for the vector from current pos (px, py) to laser detected (px', py'),
    # with the angle denoted by the index of the current laser_data entry,
    # create like 20 points (experiment), and send each of these points to the Grid class,
    # and update the corresponding gx, gy of the grid at that point with a 0. value
    # note the max range of 3.5 metres for any 'inf' laser scan results

    # todo 2.
    # integrate this with the .pgm map to have the obstacles
    # i think i can get it from occupancygrid? that way it may be able to dynamically update as the
    # robot updates its localisation etc if there's some sort of callback to it?


if __name__ == '__main__':

    rospy.init_node('frontier_map', anonymous=True)

    camera_data = rospy.wait_for_message('camera/rgb/camera_info', CameraInfo, timeout=5)
    laser_range_max = rospy.wait_for_message('scan', LaserScan, timeout=5).range_max  # max distance that laser detects
    fov = get_fov(camera_data)  # the field of view of the camera

    rospy.loginfo('laser range: ' + str(laser_range_max))

    grid = Grid()
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
