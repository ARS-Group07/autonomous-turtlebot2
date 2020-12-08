import rospy
import thread

from robot import Robot
from behaviours import *
from status import StatusWindow

class Sequencer:
    def __init__(self, robot):
        self.robot = robot
        self.cycles = 0
        self.status_window = StatusWindow(robot)
        self.current_behaviour = Exploration()
        self.sequence_hz = 25

    def sequence(self, robot):
        rate = rospy.Rate(self.sequence_hz)

        self.cycles = self.cycles + 1
        while not rospy.is_shutdown():
            self.cycles += 1
            # rospy.loginfo("Behaviour: " + self.current_behaviour.name)
            self.current_behaviour.act(robot, self)
            self.status_window.update(self.cycles)
            rate.sleep()

    """def warn_idle(self):
        if isinstance(self.current_behaviour, Exploration):
            self.current_behaviour = ExplorationUnsticking(self)"""

    """def unstuck(self):
        self.current_behaviour = Exploration()"""

    # Call of this function may come from various threads (i.e. topics from other nodes)
    def begin_homing(self, object_id, homing_ang_vel):
        self.robot.cancel_nav_goals()

        if self.robot.is_object_found(object_id):
            return

        """or isinstance(self.current_behaviour, ExplorationUnsticking)"""
        if isinstance(self.current_behaviour, Exploration):
            rospy.loginfo("[HOMING] Homing towards obj " + str(object_id))

            self.current_behaviour = Homing(self, self.robot.laser_angles)
            self.current_behaviour.current_object_type = object_id
            self.current_behaviour.ang_vel = homing_ang_vel
        elif isinstance(self.current_behaviour, Homing):
            # Only update the angular velocity if this function call is for the same object type we've been homing towards
            if self.current_behaviour.current_object_type == object_id:
                # TODO SOME SORT OF PRIORITY SYSTEM USING THE OBJECT ID IN CASE BOTH ARE IN THE SAME IMAGE FRAME
                self.current_behaviour.current_object_type = object_id
                self.current_behaviour.ang_vel = homing_ang_vel

    def finished_homing(self):
        self.current_behaviour = Exploration()
