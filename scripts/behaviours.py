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
        self.last_goal_x = 0
        self.last_goal_y = 0
        self.idle_resend = False

    def act(self, robot, sequencer):
        aoif = robot.aoif
        if sequencer.cycles % sequencer.sequence_hz == 0:
            wx, wy = robot.grid.to_world(aoif.closest_cx / aoif.scale,
                                         aoif.closest_cy / aoif.scale)

            if self.last_goal_x == aoif.closest_cx and self.last_goal_y == aoif.closest_cy:
                rospy.loginfo('Contour unchanged: randomizing slightly (cycles=' + str(sequencer.cycles) + ')')
                wx += round(random.random() / 2, 1) - 0.3
                wy += round(random.random() / 2, 1) - 0.3

            self.last_goal_x = aoif.closest_cx
            self.last_goal_y = aoif.closest_cy
            robot.send_nav_goal(wx, -wy)
            rospy.loginfo('new nav_goal sent to ' + str(wx) + ', ' + str(wy))


class Homing(Behaviour):
    def __init__(self, sequencer, laser_angles):
        Behaviour.__init__(self, "Homing")

        self.sequencer = sequencer
        self.laser_angles = laser_angles
        self.current_object_type = -1

        self.last_goal_x = -1e6
        self.last_goal_y = -1e6
        self.target_pose = None

    def set_target(self, robot, detection_msg):
        self.current_object_type = detection_msg.id
        self.target_pose = Pose(robot.pose.px + detection_msg.x, robot.pose.py + detection_msg.y, robot.pose.yaw)

        '''# Calculate the angle from the robot to the object
        vec_to = [robot.pose.px + detection_msg.x, robot.pose.py + detection_msg.y]
        vec_from = [robot.pose.px, robot.pose.py]
        unit_to = vec_to / np.linalg.norm(vec_to)
        unit_from = vec_from / np.linalg.norm(vec_from)
        dot_product = np.dot(unit_to, unit_from)
        angle = np.arccos(dot_product)

        # Determine the distance between the robot and the object
        dist_vec = [vec_to[0] - vec_from[0], vec_to[1] - vec_from[0]]
        dist = math.sqrt(dist_vec[0] ** 2 + dist_vec[1] ** 2)

        if dist > 1.0:  # It's too far so the robot needs to move towards the object as well as specifying a rotation
            norm = [dist_vec[0] / dist, dist_vec[1] / dist]
            target_dist = dist - 1.0
            self.target_pose = Pose(robot.pose.px + target_dist * norm[0], robot.pose.py + target_dist * norm[1], angle)
        else:  # It's close enough so all we need to do is specify the angle
            self.target_pose = Pose(robot.pose.px, robot.pose.py, angle)
        '''

    def act(self, robot, sequencer):
        # TODO: Do something with move_base to move towards it
        # TODO: Check if we're sufficiently closed (angular & euclidean dist)
        if robot.pose == None or self.target_pose == None:
            return

            # Firstly check if we're close enough
        if robot.pose.dist(self.target_pose) < 0.25:  # TODO - Check if its been idle for a while?
            # Check if the angular distance is sufficient: is it looking at the object?
            if robot.pose.ang_dist(self.target_pose) < 0.2:
                robot.cancel_nav_goals()
                self.finished(robot)
                return

        if self.target_pose.px != self.last_goal_x and self.target_pose.py != self.last_goal_y:
            self.last_goal_x = self.target_pose.px
            self.last_goal_y = self.target_pose.py
            robot.send_nav_goal(self.target_pose.px, self.target_pose.py, self.target_pose.yaw)
            rospy.loginfo("Sending nav goal for homing to " + str(self.target_pose.px) + ", " + str(self.target_pose.py))

    def finished(self, robot):
        robot.set_object_found(self.current_object_type)
        self.sequencer.finished_homing()
        self.last_goal_x = -1e6
        self.last_goal_y = -1e6
