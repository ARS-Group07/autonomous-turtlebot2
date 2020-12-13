#!/usr/bin/env python2.7
import cv2
import cv_bridge
import rospy
from ars.msg import Detection
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion
import pytesseract

import depth
from detect_utils import get_detection_message, AMCLConfidenceChecker
from pose import Pose


def contrast(image):
    alpha = 3  # Simple contrast control
    beta = 70  # Simple brightness control
    out = cv2.addWeighted(image, alpha, image, 0, beta)
    return out


class TextSensor:
    def __init__(self):
        # Listen for confidence before we start detecting
        confidence_checker = AMCLConfidenceChecker('Text Detection', self.on_amcl_confidence_achieved)
        confidence_checker.listen_for_confidence()
        self.flag = False
        self.bridge = cv_bridge.CvBridge()
        self.pose = Pose()

    def on_amcl_confidence_achieved(self):
        self.depthSensor = depth.DepthSensor()
        self.image_sub_text = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_text)
        self.amcl_pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.get_amcl_data)
        self.detection_pub_text = rospy.Publisher('detection_text', Detection)

    def image_callback_text(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        depth_image = None
        if self.depthSensor.depth_img is not None:
            depth_image = self.depthSensor.depth_img.copy()

        (flag, coord) = self.detect(image)
        cx, cy = coord

        # if object has been detected
        if flag:

            # get message containing object's absolute world co-ordinates from the current pose, cx,
            # cy and depth image
            detection_msg = get_detection_message(self.pose, cx, cy, depth_image, obj=3)

            if detection_msg:
                self.detection_pub_text.publish(detection_msg)

    def detect(self, image):

        self.flag = False

        image = contrast(image)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        custom_oem_psm_config = r'--psm 6'

        text2 = pytesseract.image_to_data(image, config=custom_oem_psm_config, output_type=pytesseract.Output.DICT)
        print("reading result")

        for i in range(0, len(text2["text"])):
            if int(text2["conf"][i]) >= 80:
                print("Sure Reading")
                print(int(text2["conf"][i]))
                print(text2["text"][i])
            if text2["text"][i] == "5".encode() and int(text2["conf"][i]) >= 80:
                print("We did it bois")
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
