import numpy as np
import math
import time
from ars.msg import Detection
from pose import Pose


def get_detection_message(original_pose, cx, cy, depth_image=None, obj=None):
    """ Takes in information from sensors and forms an absolute world location of the detected object, and creates a
    message. """

    if depth_image is not None:
        frame = np.asarray(depth_image)
        depth = frame[cy][cx]

        alpha = np.deg2rad(abs((cx * 4 * 60 / 1920) - 30))

        if math.isnan(depth):
            return False

        x = depth * math.tan(alpha)
        z = depth

        if cx * 4 < 960:
            x = -x

        pose2 = Pose(z, x, alpha)
        goal_pose = original_pose.locate(pose2)

        detection_msg = Detection()
        detection_msg.id = obj  # the id of the object, eg green box = 0 here
        detection_msg.timestamp = time.time()
        detection_msg.x = goal_pose.px
        detection_msg.y = goal_pose.py
        detection_msg.z = goal_pose.yaw

        return detection_msg

    return False
