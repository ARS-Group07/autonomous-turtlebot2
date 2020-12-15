#!/usr/bin/env python2.7


import numpy as np
import rospy
import math
import time

from ars.msg import Detection
from pose import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped

#Utilities for object detection classes



#detection message callback 
def get_detection_message(original_pose, cx, cy, depth_image=None, obj=None):
    """ Takes in information from sensors and forms an absolute world location of the detected object, and creates a
    message. """

    
    if depth_image is not None:
        frame = np.asarray(depth_image)
        depth = frame[cy][cx]
        #from position of pixel and fov of camera, calculate the angle  between front of camera and vector towards object
        #note that while depth and regular camera report different fovs, this seems to be incorrect (?) in simulation environment
        alpha = np.deg2rad(abs((cx * 60 / 1920) - 30))

        if math.isnan(depth):
            return False

        x = depth * math.tan(alpha)
        z = depth

        if cx < 960:
            x = -x

        if obj == 2:
            # check if the object is at high enough point for blue things (mailbox)
            beta = np.deg2rad(abs((cy * 45 / 1080) - 22.5))
            y = depth * math.tan(beta)
            if y < 0.4:
                return False

        pose2 = Pose(x, z, alpha)
        goal_pose = original_pose.locate(pose2)

        detection_msg = Detection()
        detection_msg.id = obj  # the id of the object, eg green box = 0 here
        detection_msg.timestamp = time.time()
        detection_msg.x = goal_pose.px
        detection_msg.y = goal_pose.py
        detection_msg.z = goal_pose.yaw

        return detection_msg

    return False

#checks confidence of AMCL to make sure robot is localized 
class AMCLConfidenceChecker:
    def __init__(self, checking_for, callback):
        self.checking_for = checking_for
        self.callback = callback
        self.listening = False

    def listen_for_confidence(self):
        if not self.listening:
            self.listening = True
            self.amcl_pose_subscriber = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.get_confidence)

    def get_confidence(self, msg):
        # flattened  6x6 covariance matrix for x, y, z, ang_x, ang_y, ang_z from PoseWithCovarianceStamped
        covariance_mx = msg.pose.covariance

        # get variance values for x, y and theta position to determine robot confidence in positioning
        x_var = covariance_mx[0]
        y_var = covariance_mx[7]
        theta_var = covariance_mx[35]

        rospy.loginfo('Location variance check: x var: %.4f, y var: %.4f, theta var: %.4f' % (x_var, y_var, theta_var))

        # variance = 0.01 means average of estimates squared distances from the mean estimate = 0.01m^2
        # so average distance is around 0.1m (apply sq. root)
        # assuming Gaussian distribution this is confidence margin of around +-10cm in x and y directions
        # for theta this is around +-6 degrees once converted from radians
        if x_var < 0.02 and y_var < 0.02 and theta_var < 0.02:
            rospy.loginfo('AMCL CONFIDENCE ACHIEVED FOR ' + self.checking_for + '! x, y and theta variances < 0.02')
            self.amcl_pose_subscriber.unregister()
            self.callback()
