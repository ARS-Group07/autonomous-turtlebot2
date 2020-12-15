#!/usr/bin/env python2.7
from robot import Robot
import cv2
import numpy as np
import time
from behaviours import *

# The window, updated by the Sequencer, that shows the current status & objective of the program
class StatusWindow:
    def __init__(self, robot, time_started):
        self.robot = robot
        self.time_started = time_started
        cv2.namedWindow('Status', 1)

    def get_time_elapsed(self):
        now = time.time()
        diff = round(now - self.time_started, 2)
        return str(str(diff) + " seconds")

    def update(self, cycle_count):
        # The blank canvas to show
        image = np.ones([750, 600, 3]) * 255
        sequencer = self.robot.sequencer

        # Time-related metrics
        elapsed = ['Elapsed: ' + self.get_time_elapsed()]
        cycles = ['Cycles: ' + str(cycle_count)]
        # Robot current position (from AMCL)
        odom = ['X: ' + str(round(self.robot.pose.px, 2)), 'Y: ' + str(round(self.robot.pose.py, 2))]
        # Current behaviour status
        behaviour = ['Behaviour: ' + sequencer.current_behaviour.name]
        if isinstance(sequencer.current_behaviour, Exploration):
            behaviour = behaviour + ['  Towards: (' + str(sequencer.current_behaviour.last_goal_wx) + ", "
                                     + str(sequencer.current_behaviour.last_goal_wy) + ")"]
        elif isinstance(sequencer.current_behaviour, Homing):
            behaviour = behaviour + [' obj_id: ' + str(sequencer.current_behaviour.current_object_id)]
            behaviour = behaviour + [' goal_x: ' + str(sequencer.current_behaviour.target_pose.px)]
            behaviour = behaviour + [' goal_y: ' + str(sequencer.current_behaviour.target_pose.py)]
        behaviour = behaviour + ['Idle: ' + str(self.robot.idle_tracker.idle)]

        # Object statuses (whether found, how many times seen and approximated location)
        green_seen_at = self.robot.seen_store.positions[0]
        green_seen_at[0] = round(green_seen_at[0], 3)
        green_seen_at[1] = round(green_seen_at[1], 1)
        red_seen_at = self.robot.seen_store.positions[1]
        red_seen_at[0] = round(red_seen_at[0], 3)
        red_seen_at[1] = round(red_seen_at[1], 1)
        blue_seen_at = self.robot.seen_store.positions[2]
        blue_seen_at[0] = round(blue_seen_at[0], 3)
        blue_seen_at[1] = round(blue_seen_at[1], 1)
        white_seen_at = self.robot.seen_store.positions[3]
        white_seen_at[0] = round(white_seen_at[0], 3)
        white_seen_at[1] = round(white_seen_at[1], 1)
        objects = ['Objects found:',
                   '  Green cuboid: ' + str(self.robot.is_object_found(0)),
                   '      Seen x' + str(self.robot.get_times_seen(0)),
                   '      Seen at (' + str(green_seen_at) + ')',
                   '  Red hydrant: ' + str(self.robot.is_object_found(1)),
                   '      Seen x' + str(self.robot.get_times_seen(1)),
                   '      Seen at (' + str(red_seen_at) + ')',
                   '  Blue mailbox: ' + str(self.robot.is_object_found(2)),
                   '      Seen x' + str(self.robot.get_times_seen(2)),
                   '      Seen at (' + str(blue_seen_at) + ')',
                   '  White cube: ' + str(self.robot.is_object_found(3)),
                   '      Seen x' + str(self.robot.get_times_seen(3)),
                   '      Seen at (' + str(white_seen_at) + ')',]

        offset = 30
        x, y = 10, 30
        # Write all of the lines of text we just generated to the blank canvas
        for idx, lbl in enumerate(elapsed + cycles + odom + behaviour + objects):
            cv2.putText(image, str(lbl), (x, y + offset * idx), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

        cv2.imshow("Status", image)
        cv2.waitKey(3)
