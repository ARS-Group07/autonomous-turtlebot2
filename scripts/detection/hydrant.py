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
from detection_paths import Paths

CONFIDENCE_THRESHOLD = 0.3
LABELS = open(Paths.LABELS_FILE).read().strip().split("\n")

np.random.seed(4)
COLORS = np.random.randint(0, 255, size=(len(LABELS), 3), dtype="uint8")


class HydrantDetector:
    def __init__(self):
        self.depthSensor = depth.DepthSensor()
        self.bridge = cv_bridge.CvBridge()
        self.image_sub_fire_hydrant = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_fire_hydrant)
        self.net = cv2.dnn.readNetFromDarknet(Paths.CONFIG_FILE, Paths.WEIGHTS_FILE)
        self.detection_pub_hydrant = rospy.Publisher('detection_hydrant', Detection)

    def image_callback_fire_hydrant(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (H, W) = image.shape[:2]
        depth_image = None

        if self.depthSensor.depth_img is not None:
            depth_image = self.depthSensor.depth_img.copy()
        timestamp = time.time()

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
                    print("Fire hydrant found")
                    alpha = np.deg2rad(abs((cx * 60 / 1920) - 30))
                    beta = np.deg2rad(abs((cy * 45 / 1080) - 22.5))

                    if depth_image is not None:
                        frame = np.asarray(depth_image)

                    depth = frame[cy][cx]

                    if math.isinf(depth):
                        rospy.loginfo('infinite depth')
                        continue

                    z = depth
                    x = depth * math.tan(alpha)
                    y = depth * math.tan(beta)
                    if cx < 960:
                        x = -x
                    if cy > 540:
                        y = -y

                    print("Object Location: x=" + str(x) + ', y=' + str(y) + ', z=' + str(z))

                    detection_msg = Detection()
                    detection_msg.id = 1
                    detection_msg.timestamp = timestamp
                    detection_msg.x = x
                    detection_msg.y = y
                    detection_msg.z = z
                    self.detection_pub_hydrant.publish(detection_msg)

        # cv2.imshow("pred", image)
        cv2.waitKey(3)


rospy.init_node('HydrantDetector')
analyzer = HydrantDetector()
rospy.spin()
