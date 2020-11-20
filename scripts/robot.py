#import follow
#from follow import Pose
#from follow import wander
import pose
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
    def __init__(self, x=0., y=0., yaw=0., sequencer = None, grid = None):
        self.pose = pose.Pose(x,y,yaw)
        # Flags / states
        self.state = "wander"
        self.flag_obstacle_front = False
        self.flag_obstacle_right = False
        self.flag_imminentobstacle = False
        self.flag_object = False
        self.sequencer = sequencer
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

        self.pose = pose.Pose(x, y, yaw)


    def get_laser_data(self, msg):
   
        laser_data = msg.ranges[:31] + msg.ranges[-30:]
        laser_data_right = msg.ranges[31:150]
        distright = min(laser_data_right)
        #rospy.loginfo('right_dist: ' + str(distright))

        dist = min(laser_data)
        #rospy.loginfo('dist: ' + str(dist))

        if distright <= 1.0:
            self.flag_obstacle_right = True
        else:
            self.flag_obstacle_right = False
        if 0.25 < dist < 0.5:
            self.flag_obstacle_front = True    
        elif dist <= 0.25:
            self.flag_imminentobstacle = True
            self.flag_obstacle_front = True 
        else: 
            self.flag_obstacle_front = False
            self.flag_imminentobstacle = False


    def get_image_data(self, msg):
        # TOD0
        test = 1

    def get_grid():
        return self.grid
