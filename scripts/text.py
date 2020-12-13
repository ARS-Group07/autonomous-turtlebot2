#!/usr/bin/env python2.7
import cv2
import cv_bridge
import rospy
from ars.msg import Detection
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion
import pytesseract
import numpy as np

import depth
from detect_utils import get_detection_message, AMCLConfidenceChecker
from pose import Pose


def contrast(image):
    alpha = 5  # Simple contrast control
    beta = 70  # Simple brightness control
    out = cv2.addWeighted(image, alpha, image, 0, beta)
    return out


class TextSensor:
    def __init__(self):
        # Listen for confidence before we start detecting
        self.flag = False
        self.bridge = cv_bridge.CvBridge()
        self.pose = Pose()
        confidence_checker = AMCLConfidenceChecker('Text Detection', self.on_amcl_confidence_achieved)
        confidence_checker.listen_for_confidence()

    def on_amcl_confidence_achieved(self):
        self.image_sub_text = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_text)
        self.depthSensor = depth.DepthSensor()
        self.amcl_pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.get_amcl_data)
        self.detection_pub_text = rospy.Publisher('detection_text', Detection)

    def image_callback_text(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = cv2.resize(image, (image.shape[1] / 5, image.shape[0] / 5))
        image = cv2.inRange(image, (0, 0, 0), (5, 5, 5))

        depth_image = None
        if self.depthSensor.depth_img is not None:
            depth_image = self.depthSensor.depth_img.copy()

        cv2.imshow("text", image)
        cv2.waitKey(3)

        (flag, coord) = self.detect(image)
        cx, cy = coord

        # if object has been detected
        if flag:

            # get message containing object's absolute world co-ordinates from the current pose, cx,
            # cy and depth image
            detection_msg = get_detection_message(self.pose, cx * 5, cy * 5, depth_image, obj=3)

            if detection_msg:
                self.detection_pub_text.publish(detection_msg)

    def detect(self, image):

        self.flag = False

        custom_oem_psm_config = r'--psm 6'
        text2 = pytesseract.image_to_data(image, config=custom_oem_psm_config, output_type=pytesseract.Output.DICT)

        rospy.loginfo('SUCCESSFUL DETECT CHECK')

        for i in range(0, len(text2["text"])):
            if int(text2["conf"][i]) >= 70 and text2["text"][i] == "5".encode():
                center = (text2["left"][i] + text2["width"][i] / 2, text2["top"][i] + text2["height"][i] / 2)
                self.flag = True

        if self.flag:
            return True, center
        else:
            return False, (0, 0)

    def get_amcl_data(self, msg):
        """ Gets predicted position data from the adaptive Monte Carlo module and uses it for the grids, etc. """
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(quarternion)
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y

        self.pose.update_pose(px, py, yaw)


rospy.init_node('TextSensor')
analyzer = TextSensor()
rospy.spin()
