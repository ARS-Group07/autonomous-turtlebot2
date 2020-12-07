#!/usr/bin/env python
import rospy
import cv2, cv_bridge
import numpy as np
import time 
import math
import depth

from sensor_msgs.msg import Image
from detection_paths import Paths

CONFIDENCE_THRESHOLD=0.3
LABELS = open(Paths.LABELS_FILE).read().strip().split("\n")

np.random.seed(4)
COLORS = np.random.randint(0, 255, size=(len(LABELS), 3),
	dtype="uint8")

class HydrantDetector:
   def __init__(self):
      self.depthSensor = depth.DepthSensor()
      self.bridge = cv_bridge.CvBridge()
      self.image_sub_firehydrant = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_firehydrant)
      self.net = cv2.dnn.readNetFromDarknet(Paths.CONFIG_FILE, Paths.WEIGHTS_FILE)

   def image_callback_firehydrant(self, msg):      
      image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
      (H, W) = image.shape[:2]
      depthimage = None
      if(self.depthSensor.depthimg is not None):
        depthimage = self.depthSensor.depthimg.copy()
      timestamp = time.time()
      #image = contrast(image)
      detection = False
      ln = self.net.getLayerNames()
      ln = [ln[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
      print("Layer Names:  ")
      print(ln)
      image_resized = cv2.resize(image, (W/4,H/4))
      blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416),swapRB=True, crop=False)
      self.net.setInput(blob)
      start = time.time()
      layerOutputs = self.net.forward(ln)
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

              cx = x + w/2
              cy = y + h/2
              if(LABELS[classIDs[i]] is LABELS[10]):
                  print("Fire hydrant found")
                  detection = True
                  alpha = np.deg2rad(abs((cx*60/1920) - 30))
                  beta = np.deg2rad(abs((cy*45/1080) - 22.5))
                  if(depthimage is not None):
                    frame = np.asarray(depthimage) 
               
                  depth = frame[cy][cx]
                  z = depth 
                  x = depth * math.tan(alpha)
                  y = depth * math.tan(beta)              
                  if(cx < 960):
                    x = -x
                  if(cy > 540):
                    y = -y 
                  print("Fire hydrant Location")
                  print(x,y,z)
              text = "{}: {:.4f}".format(LABELS[classIDs[i]], confidences[i])
              #cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,0.5, color, 2)
      cv2.imshow("pred", image)
      cv2.waitKey(3)

rospy.init_node('HydrantDetector')
analyzer = HydrantDetector()
rospy.spin()