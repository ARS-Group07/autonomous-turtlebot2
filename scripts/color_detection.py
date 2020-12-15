#!/usr/bin/env python2.7

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


class ColorDetector:
    def __init__(self):
        # Listen for confidence before we start detecting
        confidence_checker = AMCLConfidenceChecker('Color detection', self.on_amcl_confidence_achieved)
        confidence_checker.listen_for_confidence()
        self.bridge = cv_bridge.CvBridge()
        self.pose = Pose()

    def on_amcl_confidence_achieved(self):
        self.depthSensor = depth.DepthSensor()
        self.amcl_pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.get_amcl_data)

        self.image_sub_red = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_red)
        self.image_sub_green = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_green)
        self.image_sub_blue = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_blue)

        self.detection_pub_red = rospy.Publisher('detection_red', Detection, queue_size=10)
        self.detection_pub_green = rospy.Publisher('detection_green', Detection, queue_size=10)
        self.detection_pub_blue = rospy.Publisher('detection_blue', Detection, queue_size=10)

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
        # cv2.waitKey(3)

    def image_callback_blue(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (H, W) = image.shape[:2]
        depth_image = None
        if self.depthSensor.depth_img is not None:
            depth_image = self.depthSensor.depth_img.copy()

        image_resized = cv2.resize(image, (W / 4, H / 4))
        hsv = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (100, 102, 31), (120, 230, 56))
        # need to convert to bgr so we can convert to grey
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        lowest_cy = 1e6
        count = 0
        sum_x = 0

        for c in contours:
            M = cv2.moments(c)
            if M['m00'] < 100:
                continue

            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            sum_x += cx
            count += 1

            if cy < lowest_cy:
                lowest_cy = cy

        if count != 0:
            # get message containing object's absolute world co-ordinates from the current pose, cx, cy and depth image
            avg_x = sum_x / count
            detection_msg = get_detection_message(self.pose, avg_x * 4, lowest_cy * 4, depth_image, obj=2)

            if detection_msg:
                self.detection_pub_blue.publish(detection_msg)

            cv2.circle(mask, (avg_x, lowest_cy), 5, 127, -1)

        cv2.imshow("Blue detection", mask)
        cv2.waitKey(3)

    def image_callback_red(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (H, W) = image.shape[:2]
        depth_image = None
        if self.depthSensor.depth_img is not None:
            depth_image = self.depthSensor.depth_img.copy()

        image_resized = cv2.resize(image, (W / 4, H / 4))
        hsv = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 220, int(255 * 5 / 100)), (0, 255, int(255 * 30 / 100)))
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
            if detection_msg is not False:
                self.detection_pub_red.publish(detection_msg)
                # rospy.loginfo("Sending hydrant message")

        # cv2.imshow("Red detection", mask)
        # cv2.waitKey(3)

    def get_amcl_data(self, msg):
        """ Gets predicted position data from the adaptive Monte Carlo module and uses it for the grids, etc. """
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(quarternion)
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y

        self.pose.update_pose(px, py, yaw)


rospy.init_node('ColorDetector')
analyzer = ColorDetector()
rospy.spin()
