#!/usr/bin/env python2.7
import math
import numpy as np
import random
import rospy

from geometry_msgs.msg import Twist
from pose import Pose

# Base class for sequencing behaviours
class Behaviour:
    # Static variable for child behaviours
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def __init__(self, name):
        self.name = name

    def act(self, robot, sequencer):
        print("Error: child class should override this")

# The child class for exploring the map using the closest contours
class Exploration(Behaviour):
    def __init__(self):
        Behaviour.__init__(self, "Exploration")
        # Last goal in terms of grid coordinates
        self.last_goal_gx = 0
        self.last_goal_gy = 0
        # Last goal in terms of world coordinates
        self.last_goal_wx = 0
        self.last_goal_wy = 0

    def act(self, robot, sequencer):
        aoif = robot.aoif
        if (sequencer.cycles % sequencer.sequence_hz) == 0:
            wx, wy = robot.grid.to_world(aoif.closest_cx / aoif.scale, aoif.closest_cy / aoif.scale)

            # If this goal is the same as the last goal, then let's randomise slightly
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
            # Invert the Y because the grid is inverted
            robot.send_nav_goal(wx, -wy - 0.5)
            rospy.loginfo('new nav_goal sent to ' + str(wx) + ', ' + str(wy))

# The child class for homing towards a detected object
class Homing(Behaviour):
    def __init__(self, sequencer):
        Behaviour.__init__(self, "Homing")

        self.sequencer = sequencer
        self.current_object_id = -1
        self.target_pose = None

    # Set the target that we're homing towards
    def set_target(self, object_id, x, y, yaw):
        self.current_object_id = object_id
        self.target_pose = Pose(x, y, yaw)

    def act(self, robot, sequencer):
        if robot.pose is None or self.target_pose is None:
            return

        # Firstly check if we're close enough and cancel if it is
        if robot.pose.dist(self.target_pose) < 1.0:
            # Check if the angular distance is sufficient: is it looking at the object?
            rospy.loginfo('WITHIN TARGET DISTANCE OF OBJECT')
            rospy.loginfo('WITHIN TARGET DISTANCE OF OBJECT')
            rospy.loginfo('WITHIN TARGET DISTANCE OF OBJECT')
            robot.cancel_nav_goals()
            robot.set_object_found(self.current_object_id)
            self.sequencer.finished_homing()
            return

        # Send the nav goal to move towards the object
        if (sequencer.cycles % sequencer.sequence_hz) == 0:
            robot.send_nav_goal(self.target_pose.px, self.target_pose.py, self.target_pose.yaw)
            rospy.loginfo("Sending nav goal for homing to " + str(self.target_pose.px) + ", " + str(self.target_pose.py))

# The child class for reversing the turtle if it becomes stuck for any reason
class Unstick(Behaviour):
    # return_to -> the behaviour to return to
    def __init__(self, sequencer, return_to):
        Behaviour.__init__(self, "Unstick")

        self.sequencer = sequencer
        self.return_to = return_to
        self.pose_before = None
        self.direction = -0.5
        self.cycles = 0

    def act(self, robot, sequencer):
        # To prevent a stack forming of unstick behaviours
        robot.idle_tracker.flush()
        self.cycles += 1

        if not self.pose_before:
            self.pose_before = robot.pose.clone()
            robot.cancel_nav_goals()

        # Apply the linear velocity
        twist = Twist()
        twist.linear.x = self.direction
        Behaviour.velocity_publisher.publish(twist)

        # Evaluate whether it's been unsticking for too long / has moved a sufficient distance
        current_pose = robot.pose
        if current_pose.dist(self.pose_before) > 0.1 or self.cycles > 50:
            sequencer.finished_unsticking()
