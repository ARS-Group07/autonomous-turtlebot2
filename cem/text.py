#!/usr/bin/env python
import cv2
import cv_bridge
import math
import numpy as np
import time

import rospy
from sensor_msgs.msg import Image

import depth
import textdetect


def contrast(image):
    alpha = 3 # Simple contrast control
    beta = 70  # Simple brightness control
    out = cv2.addWeighted( image, alpha, image, 0, beta)
    return out

class TextSensor:
#TODO separate subscribers to other classes
   def __init__(self):
      self.depthSensor = depth.DepthSensor()
      self.bridge = cv_bridge.CvBridge()
      self.image_sub_text = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_text)
      self.td = textdetect.TextDetector()

   def image_callback_text(self,msg):
       image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
       #image = contrast(image)
       depthimage = None
       if self.depthSensor.depthimg is not None:
           depthimage = self.depthSensor.depthimg.copy()
       timestamp = time.time()
       (flag, coord) = self.td.detect(image)
       (xc, yc) = coord
       
       if(flag):
           print("success")
           print(coord)
           alpha = np.deg2rad(abs((xc*59/1920) - 29.5))
           beta = np.deg2rad(abs((yc*46/1080) - 23))
           if(depthimage is not None):
               frame = np.asarray(depthimage)
               depth = frame[yc][xc]
               z = depth 
               x = depth * math.tan(alpha)
               y = depth * math.tan(beta)
               if((xc< 960)):
                   x = -x
               if((yc > 540)):
                   y = -y
               print(" 5 Box found")
               print("Object location:")
               print(x,y,z)


rospy.init_node('TextSensor')
analyzer = TextSensor()
rospy.spin()