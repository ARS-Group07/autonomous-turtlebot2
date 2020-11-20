#import follow
#from follow import Pose
#from follow import wander
from pose import Pose
import cv2
import math
import numpy as np
import random
import rospy
import sequencer

from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from tf.transformations import euler_from_quaternion

class Robot:
    def __init__(self, x=0., y=0., yaw=0., sequencer = None, fov=3, grid = None):
        self.pose = Pose(x,y,yaw)
        self.sequencer = sequencer
        self.fov = fov
        self.grid = grid

        rospy.Subscriber('scan', LaserScan, self.get_laser_data)
        rospy.Subscriber('odom', Odometry, self.get_odom_data)
        rospy.Subscriber('camera/rgb/image_raw', Image, self.get_image_data)
            
    def get_odom_data(self, msg):
        """ Converts to euler angles, saves the received odom data as a dictionary and appends to global odom_msgs list. """
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(quarternion)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.pose = Pose(x, y, yaw)
        self.grid.update_grid(self.pose.x, self.pose.y, 'CURR')


    def get_laser_data(self, msg):
        point_density = 4  # eg sample a laser beam every 3 degrees
        plot_density = 0.125  # eg plot a probability point every 0.125 metres

        laser_range = int(self.fov / 2.)  # the positive and negative fov angles
        laser_angles = list(range(-laser_range, 0, point_density)) + list(range(0, laser_range, point_density))
        laser_distances = [msg.ranges[i] for i in laser_angles]  # sample the laser data every (point_density) points

        # iterate through all those laser readings
        for angle, dist in zip(laser_angles, laser_distances):
            if math.isinf(dist):  # if laser reads inf distance, clip to the laser's actual max range
                dist = laser_range_max

            plot_points = self.pose.plot_points_from_laser(angle, dist, plot_density)  # convert to a list of scanned points
            for plot_point in plot_points:
                self.grid.update_grid(plot_point[0], plot_point[1], 'NO_OBJ')  # update the grid at each point

    def get_image_data(self, msg):
        # TOD0
        test = 1

    def get_grid():
        return self.grid
