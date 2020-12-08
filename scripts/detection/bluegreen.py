#!/usr/bin/env python2.7
import math
import numpy as np
import time

import cv2
import cv_bridge
import rospy
from ars.msg import Detection
from sensor_msgs.msg import Image

import depth


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

        self.detection_pub_green = rospy.Publisher('detection_green', Detection)
        self.detection_pub_blue = rospy.Publisher('detection_blue', Detection)

    def image_callback_green(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (H, W) = image.shape[:2]
        depth_image = None
        if self.depthSensor.depth_img is not None:
            depth_image = self.depthSensor.depth_img.copy()
        timestamp = time.time()

        image_resized = cv2.resize(image, (W / 4, H / 4))
        hsv = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (36, 25, 25), (70, 255, 255))
        # need to convert to bgr so we can convert to grey
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            M = cv2.moments(c)
            if M['m00'] < 30:
                continue

            print(M['m00'])
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            alpha = np.deg2rad(abs((cx * 4 * 60 / 1920) - 30))
            beta = np.deg2rad(abs((cy * 4 * 45 / 1080) - 22.5))

            if depth_image is not None:
                frame = np.asarray(depth_image)
                depth = frame[cy * 4][cx * 4]

                if math.isinf(depth):
                    rospy.loginfo('infinite depth')
                    continue

                z = depth
                x = depth * math.tan(alpha)
                y = depth * math.tan(beta)

                if cx * 4 < 960:
                    x = -x
                if cy * 4 > 540:
                    y = -y

                detection_msg = Detection()
                detection_msg.id = 0
                detection_msg.timestamp = timestamp
                detection_msg.x = x
                detection_msg.y = y
                detection_msg.z = z

                self.detection_pub_green.publish(detection_msg)
                print("Green Object Found")
                print("Object Location: x=" + str(x) + ', y=' + str(y) + ', z=' + str(z))

            cv2.circle(mask, (cx, cy), 5, 127, -1)

        # cv2.imshow("masked", mask)
        cv2.waitKey(3)

    def image_callback_blue(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (H, W) = image.shape[:2]
        depth_image = None
        if self.depthSensor.depth_img is not None:
            depth_image = self.depthSensor.depth_img.copy()
        timestamp = time.time()

        image_resized = cv2.resize(image, (W / 4, H / 4))
        hsv = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (100, 150, 0), (150, 255, 255))
        # need to convert to bgr so we can convert to grey
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            M = cv2.moments(c)
            if M['m00'] < 30:
                continue

            print(M['m00'])
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            alpha = np.deg2rad(abs((cx * 4 * 60 / 1920) - 30))
            beta = np.deg2rad(abs((cy * 4 * 45 / 1080) - 22.5))

            if depth_image is not None:
                frame = np.asarray(depth_image)
                depth = frame[cy * 4][cx * 4]

                if math.isinf(depth):
                    rospy.loginfo('infinite depth')
                    continue

                z = depth
                x = depth * math.tan(alpha)
                y = depth * math.tan(beta)

                if cx * 4 < 960:
                    x = -x
                if cy * 4 > 540:
                    y = -y

                if y > 0.5 and M['m00'] >= 10:
                    print("Blue object found in high position(postbox)")
                    detection_msg = Detection()
                    detection_msg.id = 2
                    detection_msg.timestamp = timestamp
                    detection_msg.x = x
                    detection_msg.y = y
                    detection_msg.z = z
                    self.detection_pub_blue.publish(detection_msg)
                    print("Object Location: x=" + str(x) + ', y=' + str(y) + ', z=' + str(z))

            cv2.circle(mask, (cx, cy), 5, 127, -1)

        # cv2.imshow("masked2", mask)
        cv2.waitKey(3)


rospy.init_node('BlueGreenDetector')
analyzer = BlueGreenDetector()
rospy.spin()
