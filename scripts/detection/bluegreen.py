#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np
import math
import depth
import time

class Analyzer:
#TODO separate subscribers to other classes
   def __init__(self):
      self.depthSensor = depth.DepthSensor()
      self.bridge = cv_bridge.CvBridge()
      cv2.namedWindow("original", 1)
      self.image_sub_firehydrant = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_firehydrant)
      self.image_sub_depth = rospy.Subscriber('camera/depth/image_raw', Image, self.image_callback_depth)
      self.image_sub_text = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_text)
      self.image_sub_green = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_green)
      self.image_sub_blue = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_blue)

class BlueGreenDetector:
   def __init__(self):
      self.depthSensor = depth.DepthSensor()
      self.bridge = cv_bridge.CvBridge()
      self.image_sub_green = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_green)
      self.image_sub_blue = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback_blue)

   def image_callback_green(self,msg):
       image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
       (H, W) = image.shape[:2]
       depthimage = None
       if(self.depthSensor.depthimg is not None):
        depthimage = self.depthSensor.depthimg.copy()
       timestamp = time.time()
       image_resized = cv2.resize(image, (W/4,H/4))
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
               if(len(approx)) is 4:
                   detection = True
               cx = int(M['m10']/M['m00'])
               cy = int(M['m01']/M['m00'])
               alpha = np.deg2rad(abs((cx*4*60/1920) - 30))
               beta = np.deg2rad(abs((cy*4*45/1080) - 22.5))
               if(depthimage is not None):
                   frame = np.asarray(depthimage) 
                   
                   depth = frame[cy*4][cx*4]
                   z = depth 
                   x = depth * math.tan(alpha)
                   y = depth * math.tan(beta)              
                   if(cx*4 < 960):
                       x = -x
                   if(cy*4 > 540):
                       y = -y 
                    
                   print("Green Object Found")
                   print("Object Location :")
                   print(x,y,z)

               
           else:
               cx, cy = 0, 0
               detection = False


           cv2.circle(mask, (cx, cy), 5, 127, -1)


       cv2.imshow("masked", mask)
       cv2.waitKey(3)


   def image_callback_blue(self,msg):
       image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
       #image = cv2.blur(image,(5,5))
       (H, W) = image.shape[:2]
       depthimage = None
       if(self.depthSensor.depthimg is not None):
        depthimage = self.depthSensor.depthimg.copy()
       timestamp = time.time()
       image_resized = cv2.resize(image, (W/4,H/4))
       (h_resized, w_resized) = image_resized.shape[:2]
       hsv = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)

       
       mask = cv2.inRange(hsv, (100,150,0), (150,255,255))
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
               if(len(approx)) is 4:
                   detection = True
               cx = int(M['m10']/M['m00'])
               cy = int(M['m01']/M['m00'])
               alpha = np.deg2rad(abs((cx*4*60/1920) - 30))
               beta = np.deg2rad(abs((cy*4*45/1080) - 22.5))
               if(depthimage is not None):
                   frame = np.asarray(depthimage) 
                   
                   depth = frame[cy*4][cx*4]
                   z = depth 
                   x = depth * math.tan(alpha)
                   y = depth * math.tan(beta)              
                   if(cx*4 < 960):
                       x = -x
                   if(cy*4 > 540):
                       y = -y 
    
                   if( y > 0.5 and M['m00'] >= 10 ):
                       print("Blue object found in high position(postbox)")
                   print("Object Location")
                   print(x,y,z)
           else:
               cx, cy = 0, 0
               detection = False


               
           cv2.circle(mask, (cx, cy), 5, 127, -1)


       cv2.imshow("masked2", mask)
       cv2.waitKey(3)
rospy.init_node('BlueGreenDetector')
analyzer = BlueGreenDetector()
rospy.spin()