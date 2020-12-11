#!/usr/bin/env python2.7
import math
import numpy as np
import random

import rospy
from geometry_msgs.msg import Twist

from pose import Pose


class Behaviour:
    # Static variable
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def __init__(self, name):
        self.name = name

    def act(self, robot, sequencer):
        print("Error: child class should override this")


class Exploration(Behaviour):
    def __init__(self):
        Behaviour.__init__(self, "Exploration")
        # Last goal in terms of grid coordinates
        self.last_goal_gx = 0
        self.last_goal_gy = 0
        # Last goal in terms of world coordinates
        self.last_goal_wx = 0
        self.last_goal_wy = 0
        self.idle_resend = False

    def act(self, robot, sequencer):
        aoif = robot.aoif
        if sequencer.cycles % sequencer.sequence_hz == 0:
            wx, wy = robot.grid.to_world(aoif.closest_cx / aoif.scale, aoif.closest_cy / aoif.scale)

            if self.last_goal_gx == aoif.closest_cx and self.last_goal_gy == aoif.closest_cy:
                rospy.loginfo('Contour unchanged: randomizing slightly (cycles=' + str(sequencer.cycles) + ')')
                wx += round(random.random() / 2, 1) - 0.3
                wy += round(random.random() / 2, 1) - 0.3

            # Set the last goal (grid coords)
            self.last_goal_gx = aoif.closest_cx
            self.last_goal_gy = aoif.closest_cy
            # Set the last goal (world coords)
            self.last_goal_wx = wx
            self.last_goal_wy = wy
            robot.send_nav_goal(wx, -wy)
            rospy.loginfo('new nav_goal sent to ' + str(wx) + ', ' + str(wy))


class Homing(Behaviour):
    def __init__(self, sequencer, laser_angles):
        Behaviour.__init__(self, "Homing")

        self.sequencer = sequencer
        self.laser_angles = laser_angles
        self.current_object_id = -1

        self.target_pose = None

    def set_target(self, object_id, x, y, yaw):
        self.current_object_id = object_id
        self.target_pose = Pose(x, y, yaw)

    def act(self, robot, sequencer):
        # TODO: Do something with move_base to move towards it
        # TODO: Check if we're sufficiently closed (angular & euclidean dist)
        if robot.pose is None or self.target_pose is None:
            return

            # Firstly check if we're close enough
        if robot.pose.dist(self.target_pose) < 1.0:  # TODO - Check if its been idle for a while?
            # Check if the angular distance is sufficient: is it looking at the object?
            rospy.loginfo('WITHIN TARGET DISTANCE OF OBJECT')
            rospy.loginfo('WITHIN TARGET DISTANCE OF OBJECT')
            rospy.loginfo('WITHIN TARGET DISTANCE OF OBJECT')
            robot.cancel_nav_goals()
            self.finished(robot)
            return

        if sequencer.cycles % sequencer.sequence_hz == 0:
            self.last_goal_x = self.target_pose.px
            self.last_goal_y = self.target_pose.py
            robot.send_nav_goal(self.target_pose.px, self.target_pose.py, self.target_pose.yaw)
            rospy.loginfo("Sending nav goal for homing to " + str(self.target_pose.px) + ", " + str(self.target_pose.py))

            # TODO Fix it getting stuck while homing

    def finished(self, robot):
        robot.set_object_found(self.current_object_id)
        self.sequencer.finished_homing()
        self.last_goal_x = -1e6
        self.last_goal_y = -1e6
