#!/usr/bin/env python2.7
import cv_bridge
import rospy
from sensor_msgs.msg import Image


class DepthSensor:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub_depth = rospy.Subscriber('camera/depth/image_raw', Image, self.image_callback_depth)
        self.depth_img = None

    def image_callback_depth(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        self.depth_img = cv_image
