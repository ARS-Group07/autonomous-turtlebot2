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
import textdetect


def contrast(image):
    alpha = 3  # Simple contrast control
    beta = 70  # Simple brightness control
    out = cv2.addWeighted(image, alpha, image, 0, beta)
    return out


class TextSensor:
    # TODO separate subscribers to other classes
    def __init__(self):
        self.depthSensor = depth.DepthSensor()
        self.bridge = cv_bridge.CvBridge()
        self.image_sub_text = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_text)
        self.td = textdetect.TextDetector()
        self.detection_pub_text = rospy.Publisher('detection_text', Detection)

    def image_callback_text(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        depth_image = None
        if self.depthSensor.depth_img is not None:
            depth_image = self.depthSensor.depth_img.copy()

        timestamp = time.time()
        (flag, coord) = self.td.detect(image)
        (xc, yc) = coord

        if flag:
            print("success")
            print(coord)

            alpha = np.deg2rad(abs((xc * 59 / 1920) - 29.5))
            beta = np.deg2rad(abs((yc * 46 / 1080) - 23))

            if depth_image is not None:
                frame = np.asarray(depth_image)
                depth = frame[yc][xc]

                if math.isinf(depth):
                    rospy.loginfo('infinite depth')
                    return

                z = depth
                x = depth * math.tan(alpha)
                y = depth * math.tan(beta)

                if xc < 960:
                    x = -x
                if yc > 540:
                    y = -y

                print("5 Box found")
                print("Object Location: x=" + str(x) + ', y=' + str(y) + ', z=' + str(z))

                detection_msg = Detection()
                detection_msg.id = 3
                detection_msg.timestamp = timestamp
                detection_msg.x = x
                detection_msg.y = y
                detection_msg.z = z
                self.detection_pub_text.publish(detection_msg)

rospy.init_node('TextSensor')
analyzer = TextSensor()
rospy.spin()
