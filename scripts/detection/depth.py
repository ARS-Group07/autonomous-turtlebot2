#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np
import math
import time

class DepthSensor:
   def __init__(self):

      self.bridge = cv_bridge.CvBridge()
      self.image_sub_depth = rospy.Subscriber('camera/depth/image_raw', Image, self.image_callback_depth)
      self.depthimg = None

   def image_callback_depth(self, msg):       
       cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
       timestamp = time.time()
       frame = np.asarray(cv_image)
       self.depthimg = cv_image

       cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
       cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
       cv_image_resized = cv2.resize(cv_image_norm, (1920,1080), interpolation = cv2.INTER_CUBIC)
       cv2.imshow("Depth", cv_image_resized)
       cv2.waitKey(3)

