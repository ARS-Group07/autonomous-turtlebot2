#!/usr/bin/env python2.7
import cv2
import cv_bridge
import math
import numpy as np
import time

import rospy
from sensor_msgs.msg import Image

import textdetect


LABELS_FILE = '/home/y4tsu/catkin_ws/src/ars/src/data/coco.names'
CONFIG_FILE = '/home/y4tsu/catkin_ws/src/ars/src/cfg/yolov3.cfg'
WEIGHTS_FILE = '/home/y4tsu/catkin_ws/src/ars/src/yolov3.weights'

CONFIDENCE_THRESHOLD = 0.3
LABELS = open(LABELS_FILE).read().strip().split("\n")

np.random.seed(4)
COLORS = np.random.randint(0, 255, size=(len(LABELS), 3),
                           dtype="uint8")

net = cv2.dnn.readNetFromDarknet(CONFIG_FILE, WEIGHTS_FILE)

td = textdetect.TextDetector()


def contrast(image):
    alpha = 3  # Simple contrast control
    beta = 70  # Simple brightness control
    out = cv2.addWeighted(image, alpha, image, 0, beta)
    return out


class Analyzer:
    # TODO separate subscribers to other classes
    def __init__(self):
        self.Robot = None
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("original", 1)
        self.image_sub_firehydrant = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_firehydrant)
        self.image_sub_depth = rospy.Subscriber('camera/depth/image_raw', Image, self.image_callback_depth)
        self.image_sub_text = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_text)
        self.image_sub_green = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_green)
        self.image_sub_blue = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_blue)

        self.depthimg = None
        self.flag = False

    # Obtain depth image
    def image_callback_depth(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        timestamp = time.time()
        frame = np.asarray(cv_image)
        self.depthimg = cv_image
        cv_image_array = np.array(cv_image, dtype=np.dtype('f8'))
        cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
        cv_image_resized = cv2.resize(cv_image_norm, (1920, 1080), interpolation=cv2.INTER_CUBIC)
        cv2.imshow("Depth", cv_image_resized)
        cv2.waitKey(3)

    def image_callback_text(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        depthimage = None
        if self.depthimg is not None:
            depthimage = self.depthimg.copy()
        timestamp = time.time()
        (flag, coord) = td.detect(image)
        (xc, yc) = coord

        if flag:
            print("success")
            print(coord)
            alpha = np.deg2rad(abs((xc * 60 / 1920) - 30))
            beta = np.deg2rad(abs((yc * 45 / 1080) - 22.5))
            if (depthimage is not None):
                frame = np.asarray(depthimage)
                depth = frame[yc][xc]
                z = depth
                x = depth * math.tan(alpha)
                y = depth * math.tan(beta)
                if ((xc < 960)):
                    x = -x
                if ((yc > 540)):
                    y = -y
                print(" 5 Box found")
                print("Object location:")
                print(x, y, z)

    def image_callback_green(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (H, W) = image.shape[:2]
        depthimage = None
        if (self.depthimg is not None):
            depthimage = self.depthimg.copy()
        timestamp = time.time()
        image_resized = cv2.resize(image, (W / 4, H / 4))
        (h_resized, w_resized) = image_resized.shape[:2]
        hsv = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (36, 25, 25), (70, 255, 255))
        # need to convert to bgr so we can convert to grey
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            detection = False
            M = cv2.moments(c)
            if M['m00'] >= 30:

                print(M['m00'])
                peri = cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, 0.04 * peri, True)
                rospy.loginfo('Contours shape: ' + str(len(approx)))
                if (len(approx)) is 4:
                    detection = True
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                alpha = np.deg2rad(abs((cx * 4 * 60 / 1920) - 30))
                beta = np.deg2rad(abs((cy * 4 * 45 / 1080) - 22.5))
                if (depthimage is not None):
                    frame = np.asarray(depthimage)

                    depth = frame[cy * 4][cx * 4]
                    z = depth
                    x = depth * math.tan(alpha)
                    y = depth * math.tan(beta)
                    if cx * 4 < 960:
                        x = -x
                    if cy * 4 > 540:
                        y = -y

                    print("Green Object Found")
                    print("Object Location :")
                    print(x, y, z)

            else:
                cx, cy = 0, 0

            cv2.circle(mask, (cx, cy), 5, 127, -1)

        cv2.imshow("masked", mask)
        cv2.waitKey(3)

    def image_callback_blue(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # image = cv2.blur(image,(5,5))
        (H, W) = image.shape[:2]
        depthimage = None
        if self.depthimg is not None:
            depthimage = self.depthimg.copy()
        timestamp = time.time()
        image_resized = cv2.resize(image, (W / 4, H / 4))
        (h_resized, w_resized) = image_resized.shape[:2]
        hsv = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, (100, 150, 0), (150, 255, 255))
        # need to convert to bgr so we can convert to grey
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            detection = False
            M = cv2.moments(c)
            if M['m00'] >= 30:

                print(M['m00'])
                peri = cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, 0.04 * peri, True)
                rospy.loginfo('Contours shape: ' + str(len(approx)))
                if (len(approx)) is 4:
                    detection = True
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                alpha = np.deg2rad(abs((cx * 4 * 60 / 1920) - 30))
                beta = np.deg2rad(abs((cy * 4 * 45 / 1080) - 22.5))
                if depthimage is not None:
                    frame = np.asarray(depthimage)

                    depth = frame[cy * 4][cx * 4]
                    z = depth
                    x = depth * math.tan(alpha)
                    y = depth * math.tan(beta)
                    if cx * 4 < 960:
                        x = -x
                    if cy * 4 > 540:
                        y = -y

                    if y > 0.5 and M['m00'] >= 10:
                        print("Blue object found in high position(postbox)")
                    print("Object Location")
                    print(x, y, z)
            else:
                cx, cy = 0, 0
                detection = False

            cv2.circle(mask, (cx, cy), 5, 127, -1)

        cv2.imshow("masked2", mask)
        cv2.waitKey(3)

    # Detect objects in color image

    def image_callback_firehydrant(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (H, W) = image.shape[:2]
        depthimage = None
        if self.depthimg is not None:
            depthimage = self.depthimg.copy()
        timestamp = time.time()
        image = contrast(image)
        detection = False
        ln = net.getLayerNames()
        ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        print("Layer Names:  ")
        print(ln)
        image_resized = cv2.resize(image, (W / 4, H / 4))
        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        net.setInput(blob)
        start = time.time()
        layerOutputs = net.forward(ln)
        end = time.time()
        print("[INFO] YOLO took {:.6f} seconds".format(end - start))

        boxes = []
        confidences = []
        classIDs = []

        for output in layerOutputs:
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
                if (LABELS[classIDs[i]] is LABELS[10]):
                    print("Fire hydrant found")
                    detection = True
                    alpha = np.deg2rad(abs((cx * 60 / 1920) - 30))
                    beta = np.deg2rad(abs((cy * 45 / 1080) - 22.5))
                    if (depthimage is not None):
                        frame = np.asarray(depthimage)

                    depth = frame[cy][cx]
                    z = depth
                    x = depth * math.tan(alpha)
                    y = depth * math.tan(beta)
                    if (cx < 960):
                        x = -x
                    if (cy > 540):
                        y = -y
                    print("Fire hydrant Location")
                    print(x, y, z)
                text = "{}: {:.4f}".format(LABELS[classIDs[i]], confidences[i])
                # cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,0.5, color, 2)
        cv2.imshow("pred", image)
        cv2.waitKey(3)


rospy.init_node('Analyzer')
analyzer = Analyzer()
rospy.spin()
