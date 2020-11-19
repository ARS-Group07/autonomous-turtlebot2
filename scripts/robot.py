#import follow
#from follow import Pose
#from follow import wander
import pose
import cv2
import math
import numpy as np
import random
import rospy

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from tf.transformations import euler_from_quaternion

def twist_msg(lin_vel=0., ang_vel=0.):
    """ Creates a Twist object to be published, with only linear and angular velocity values set. """
    msg = Twist()
    msg.linear.x = lin_vel
    msg.linear.y = 0.
    msg.linear.z = 0.
    msg.angular.x = 0.
    msg.angular.y = 0.
    msg.angular.z = ang_vel
    return msg

class Robot:
    def __init__(self, x=0., y=0., yaw=0., sequencer = None):
        self.pose = pose.Pose(x,y,yaw)
        # Flags / states
        self.state = "wander"
        self.flag_obstacle_front = False
        self.flag_obstacle_right = False
        self.flag_imminentobstacle = False
        self.flag_object = False
        self.sequencer = sequencer

        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('scan', LaserScan, self.get_laser_data)
        rospy.Subscriber('odom', Odometry, self.get_odom_data)
        rospy.Subscriber('camera/rgb/image_raw', Image, self.get_image_data)

        
    def turn_left(self):
        self.publisher.publish(twist_msg(0., 0.2))
    def turn_right(self):
        self.publisher.publish(twist_msg(0.2, -0.2))
    def move_forward(self):
        self.publisher.publish(twist_msg(0.2, 0.))
            
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
