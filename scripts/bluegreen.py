#!/usr/bin/env python2.7
import math
import numpy as np

import cv2
import cv_bridge
import rospy
from ars.msg import Detection
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion

import depth
from detect_utils import get_detection_message
from pose import Pose


class Analyzer:
    # TODO separate subscribers to other classes
    def __init__(self):
        self.depthSensor = depth.DepthSensor()
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("original", 1)
        self.image_sub_firehydrant = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_firehydrant)
        self.image_sub_depth = rospy.Subscriber('camera/depth/image_raw', Image, self.image_callback_depth)
        self.image_sub_text = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_text)
        self.image_sub_green = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_green)
        self.image_sub_blue = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_blue)


class BlueGreenDetector:
    def __init__(self):
        self.depthSensor = depth.DepthSensor()
        self.bridge = cv_bridge.CvBridge()
        self.image_sub_green = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_green)
        self.image_sub_blue = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_blue)
        self.amcl_pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.get_amcl_data)

        self.detection_pub_green = rospy.Publisher('detection_green', Detection)
        self.detection_pub_blue = rospy.Publisher('detection_blue', Detection)
        self.pose = Pose()

    def image_callback_green(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (H, W) = image.shape[:2]
        depth_image = None
        if self.depthSensor.depth_img is not None:
            depth_image = self.depthSensor.depth_img.copy()

        image_resized = cv2.resize(image, (W / 4, H / 4))
        hsv = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (36, 25, 25), (70, 255, 255))
        # need to convert to bgr so we can convert to grey
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            M = cv2.moments(c)
            if M['m00'] < 30:
                continue

            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # get message containing object's absolute world co-ordinates from the current pose, cx, cy and depth image
            detection_msg = get_detection_message(self.pose, cx * 4, cy * 4, depth_image, obj=0)

            if detection_msg:
                self.detection_pub_green.publish(detection_msg)

            cv2.circle(mask, (cx, cy), 5, 127, -1)

        # cv2.imshow("masked", mask)
        cv2.waitKey(3)

    def image_callback_blue(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (H, W) = image.shape[:2]
        depth_image = None
        if self.depthSensor.depth_img is not None:
            depth_image = self.depthSensor.depth_img.copy()

        image_resized = cv2.resize(image, (W / 4, H / 4))
        hsv = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (100, 150, 0), (150, 255, 255))
        # need to convert to bgr so we can convert to grey
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            M = cv2.moments(c)
            if M['m00'] < 30:
                continue

            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # get message containing object's absolute world co-ordinates from the current pose, cx, cy and depth image
            detection_msg = get_detection_message(self.pose, cx * 4, cy * 4, depth_image, obj=2)

            if detection_msg:
                self.detection_pub_blue.publish(detection_msg)

            cv2.circle(mask, (cx, cy), 5, 127, -1)

        cv2.imshow("masked2", mask)
        cv2.waitKey(3)

    def get_amcl_data(self, msg):
        """ Gets predicted position data from the adaptive Monte Carlo module and uses it for the grids, etc. """
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(quarternion)
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y

        self.pose.update_pose(px, py, yaw)


rospy.init_node('BlueGreenDetector')
analyzer = BlueGreenDetector()
rospy.spin()
