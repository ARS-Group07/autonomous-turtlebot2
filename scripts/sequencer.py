#!/usr/bin/env python2.7
import rospy
from behaviours import Exploration, Homing, Unstick
from status import StatusWindow

# The central controller class for handling the behaviour-architecture
# Transitions for states (behaviours):
# Exploration -> Homing        upon object seen
# Homing      -> Homing        upon object found, but another object was seen while homing
# Homing      -> Exploration   upon object found, but no other (unfound) objects have been seen to home towards
# *           -> Unstick       upon robot being detected as idle
# Unstick     -> *             upon the robot becoming unstick, the previous state is returned to
class Sequencer:
    def __init__(self, robot, time_started):
        self.robot = robot
        self.cycles = 0 # How many times the sequencer has ran (number of times a behaviour has been called)
        self.status_window = StatusWindow(robot, time_started) # The status window
        self.current_behaviour = Exploration() # The starting behaviour
        self.sequence_hz = 25 # How often (ms) to run the sequencer

    # The function that calls the main sequencer loop. Only to be called once
    def sequence(self, robot):
        rate = rospy.Rate(self.sequence_hz)

        # Begin the sequencer loop
        self.cycles = self.cycles + 1
        while not rospy.is_shutdown():
            self.cycles += 1

            # If the robot is considered stuck, let's try unsticking it before returning to this instance of current
            # behaviour
            robot.idle_tracker.update_idle()
            if robot.idle_tracker.idle:
                rospy.loginfo("ROBOT IS STUCK")
                robot.idle_tracker.flush() # Flush the tracker so it doesn't immediately register as stuck again
                self.current_behaviour = Unstick(self, self.current_behaviour)

            # Run the current behaviour
            self.current_behaviour.act(robot, self)
            # Update the status window
            if self.cycles % 10 == 0:
                self.status_window.update(self.cycles)

            # Stop CPU burnout
            rate.sleep()

    # Attempt to home towards a detected object (from the detection message)
    def try_to_home(self, detection_msg):
        # If the robot has already been found (homed towards), let's ignore it
        if self.robot.is_object_found(detection_msg.id):
            return

        if isinstance(self.current_behaviour, Exploration):
            # If currently exploring, let's begin homing towards the object
            self.robot.cancel_nav_goals()
            rospy.loginfo("[HOMING] Homing towards obj " + str(detection_msg.id))

            self.current_behaviour = Homing(self)
            self.current_behaviour.set_target(*self.get_homing_location(detection_msg))
        elif isinstance(self.current_behaviour, Homing):
            if self.current_behaviour.current_object_id == detection_msg.id:
                # Home towards the latest known position of the object 
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
