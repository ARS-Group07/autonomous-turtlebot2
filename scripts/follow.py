#!/usr/bin/env python2.7
#
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
import robot
import pose
import sequencer


class ImgDataHandler:
    def __init__(self):
        self.bridge = CvBridge()
        self.boundaries = []
        cv2.namedWindow('window', 1)

    def get_image_data(self, msg):
        global state
        global follow_angle

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (h, w) = image.shape[:2]
        image = cv2.resize(image, (w / 4, h / 4))

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (36, 25, 25), (70, 255, 255))

        if np.any(mask != np.zeros(mask.shape[2:])):
            state = 'follow'
        else:
            state = 'walk'

        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for i, c in enumerate(contours):
            m = cv2.moments(c)
            if m['m00'] != 0:
                cx = int(m['m10'] / m['m00'])
                cy = int(m['m01'] / m['m00'])
            else:
                cx, cy = 0, 0

            if i == 0:
                err = cx - w / 8
                follow_angle = -float(err) / 100

            cv2.circle(mask, (cx, cy), 5, 127, -1)

        cv2.imshow('window', mask)
        cv2.waitKey(3)



def add_angle(angle1, angle2):
    angle = angle1 + angle2
    if angle >= math.pi:
        return -math.pi + math.fmod(angle, math.pi)
    else:
        return angle

 
def init_subscribers():
    rospy.Subscriber('camera/rgb/image_raw', Image, myRobot.idh.get_image_data)


if __name__ == '__main__':
    odom_data = {}
    dist_required = 3.
    follow_angle = 0.
    state = 'walk'

    myRobot = robot.Robot()
    
    try:
        myRobot.idh = ImgDataHandler()
        rospy.init_node('wander_node', anonymous=True)
        myRobot.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        

        init_subscribers()
        myRobot.sequencer = sequencer.Sequencer(myRobot)
        myRobot.sequencer.sequence()
        #sequencerf()
        

    except rospy.ROSInterruptException:
        rospy.loginfo('ROSInterruptException encountered at %s' % rospy.get_time())
