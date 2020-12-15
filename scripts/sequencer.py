#!/usr/bin/env python2.7
import rospy
from behaviours import Exploration, Homing, Unstick
from status import StatusWindow


class Sequencer:
    def __init__(self, robot, time_started):
        self.robot = robot
        self.cycles = 0
        self.status_window = StatusWindow(robot, time_started)
        self.current_behaviour = Exploration()
        self.sequence_hz = 25

    def sequence(self, robot):
        rate = rospy.Rate(self.sequence_hz)

        self.cycles = self.cycles + 1
        while not rospy.is_shutdown():
            self.cycles += 1

            robot.idle_tracker.update_idle()
            if robot.idle_tracker.idle:
                rospy.loginfo("ROBOT IS STUCK")
                robot.idle_tracker.flush()
                self.current_behaviour = Unstick(self, self.current_behaviour)

            self.current_behaviour.act(robot, self)
            if self.cycles % 10 == 0:
                self.status_window.update(self.cycles)

            rate.sleep()

    def try_to_home(self, detection_msg):
        if self.robot.is_object_found(detection_msg.id):
            return

        if isinstance(self.current_behaviour, Exploration):
            self.robot.cancel_nav_goals()
            rospy.loginfo("[HOMING] Homing towards obj " + str(detection_msg.id))

            self.current_behaviour = Homing(self)
            self.current_behaviour.set_target(*self.get_homing_location(detection_msg))
        elif isinstance(self.current_behaviour, Homing):
            # Only update the angular velocity if this function call is for the same object type we've been homing
            # towards
            if self.current_behaviour.current_object_id == detection_msg.id:
                # Just in case anything has changed
                self.current_behaviour.set_target(*self.get_homing_location(detection_msg))

    # Prevents using the detected location for a mailbox, but instead, uses the average seen position for it (which is
    # only updated when not homing towards it)
    def get_homing_location(self, detection_msg):
        # if detection_msg.id == 3:
        avg_pos = self.robot.seen_store.get_average_location(detection_msg.id)
        return detection_msg.id, avg_pos[0], avg_pos[1], 0
        # else:
        #     return detection_msg.id, detection_msg.x, detection_msg.y, detection_msg.z

    def is_homing_towards_mailbox(self):
        return isinstance(self.current_behaviour, Homing) and self.current_behaviour.current_object_id == 2

    def finished_homing(self):
        # Once finished homing, let's check to see if it detected any other objects while homing towards that object
        # in order to reduce overall search time
        object_id, position = self.robot.get_seen_unfound_object_position()
        if object_id == -1 or position is None:
            self.current_behaviour = Exploration()
        else:
            rospy.loginfo("Saw object while homing. Homing towards object " + str(object_id))
            self.current_behaviour.set_target(object_id, position[0], position[1], 0)

    def finished_unsticking(self):
        self.current_behaviour = self.current_behaviour.return_to
