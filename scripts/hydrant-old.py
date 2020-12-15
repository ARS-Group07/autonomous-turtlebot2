#!/usr/bin/env python2.7
###############################################################
###############################################################
###                                                         ###
###    LEGACY CLASS - NO LONGER IN USE - PROOF OF CONCEPT   ###
###                                                         ###
###############################################################
###############################################################

import numpy as np
import time

import cv2
import cv_bridge
import rospy
from ars.msg import Detection
from sensor_msgs.msg import Image

import depth
from detection_paths import Paths
from detect_utils import get_detection_message, AMCLConfidenceChecker
from pose import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

CONFIDENCE_THRESHOLD = 0.3
LABELS = open(Paths.LABELS_FILE).read().strip().split("\n")

np.random.seed(4)
COLORS = np.random.randint(0, 255, size=(len(LABELS), 3), dtype="uint8")

#YOLO Object detection - unused due to performance
class HydrantDetector:
    def __init__(self):
        # Listen for confidence before we start detecting
        confidence_checker = AMCLConfidenceChecker('Hydrant Detection', self.on_amcl_confidence_achieved)
        confidence_checker.listen_for_confidence()

    def on_amcl_confidence_achieved(self):
        self.depthSensor = depth.DepthSensor()
        self.bridge = cv_bridge.CvBridge()
        self.pose = Pose()
        self.net = cv2.dnn.readNetFromDarknet(Paths.CONFIG_FILE, Paths.WEIGHTS_FILE)
        self.image_sub_fire_hydrant = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_fire_hydrant)
        self.amcl_pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.get_amcl_data)
        self.detection_pub_hydrant = rospy.Publisher('detection_hydrant', Detection)

    def image_callback_fire_hydrant(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (H, W) = image.shape[:2]
        depth_image = None

        if self.depthSensor.depth_img is not None:
            depth_image = self.depthSensor.depth_img.copy()

        ln = self.net.getLayerNames()
        ln = [ln[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)

        start = time.time()
        layer_outputs = self.net.forward(ln)
        end = time.time()
        print("[INFO] YOLO took {:.6f} seconds".format(end - start))

        boxes = []
        confidences = []
        classIDs = []

        for output in layer_outputs:
            for detection in output:
                scores = detection[5:]

                classID = np.argmax(scores)
                confidence = scores[classID]

                if confidence > CONFIDENCE_THRESHOLD:
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)

        idxs = cv2.dnn.NMSBoxes(boxes, confidences, CONFIDENCE_THRESHOLD, CONFIDENCE_THRESHOLD)

        if len(idxs) > 0:
            for i in idxs.flatten():
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])
                color = [int(c) for c in COLORS[classIDs[i]]]
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)

                cx = x + w / 2
                cy = y + h / 2

                if LABELS[classIDs[i]] is LABELS[10]:

                    # get message containing object's absolute world co-ordinates from the current pose, cx,
                    # cy and depth image
                    detection_msg = get_detection_message(self.pose, cx, cy, depth_image, obj=1)

                    if detection_msg:
                        self.detection_pub_hydrant.publish(detection_msg)

        # cv2.imshow("pred", image)
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
analyzer = HydrantDetector()
rospy.spin()
