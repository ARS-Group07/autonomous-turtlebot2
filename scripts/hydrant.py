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
from detect_utils import get_detection_message, AMCLConfidenceChecker
from pose import Pose

class RedDetector:
    def __init__(self):
        # Listen for confidence before we start detecting
        confidence_checker = AMCLConfidenceChecker('Hydrant Detection', self.on_amcl_confidence_achieved)
        confidence_checker.listen_for_confidence()

    def on_amcl_confidence_achieved(self):
        self.depthSensor = depth.DepthSensor()
        self.bridge = cv_bridge.CvBridge()
        self.pose = Pose()

        self.image_sub_red = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_red)
        self.amcl_pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.get_amcl_data)

        self.detection_pub_hydrant = rospy.Publisher('detection_hydrant', Detection)

    def image_callback_red(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (H, W) = image.shape[:2]
        depth_image = None
        if self.depthSensor.depth_img is not None:
            depth_image = self.depthSensor.depth_img.copy()

        image_resized = cv2.resize(image, (W / 4, H / 4))
        hsv = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 220, int(255 * 5/100)), (0, 255, int(255 * 30/100)))
        # need to convert to bgr so we can convert to grey
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        count, sum_x, sum_y = 0, 0, 0
        for c in contours:
            M = cv2.moments(c)
            if M['m00'] < 100:
                continue

            contour_cx = int(M['m10'] / M['m00'])
            contour_cy = int(M['m01'] / M['m00'])

            count += 1
            sum_x += contour_cx
            sum_y += contour_cy

            # get message containing object's absolute world co-ordinates from the current pose, cx, cy and depth image
            cv2.circle(mask, (contour_cx, contour_cy), 5, 127, -1)

        if count > 0:
            cv2.circle(mask, (contour_cx, contour_cy), 10, 64, -1)
            cx = sum_x / count
            cy = sum_y / count
            detection_msg = get_detection_message(self.pose, cx * 4, cy * 4, depth_image, obj=1)
            self.detection_pub_hydrant.publish(detection_msg)

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


rospy.init_node('HydrantDetector')
analyzer = RedDetector()
rospy.spin()
