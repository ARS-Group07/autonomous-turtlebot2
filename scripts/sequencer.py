import rospy
import thread

from robot import Robot
from behaviours import *
from status import StatusWindow

class Sequencer:
    def __init__(self, robot, status_window):
        self.robot = robot
        self.status_window = status_window
        self.status_update_every = 5

        self.cycles = 0
        self.current_behaviour = Exploration()

    def sequence(self, robot):
        rate = rospy.Rate(25)

        while not rospy.is_shutdown():
            # rospy.loginfo("Behaviour: " + self.current_behaviour.name)
            self.current_behaviour.act(robot, self)

            self.cycles = self.cycles + 1
            if self.cycles % self.status_update_every == 0:
                self.status_window.update()

            rate.sleep()

    def warn_idle(self):
        self.current_behaviour.warn_idle()

    # Call of this function may come from various threads (i.e. topics from other nodes)
    def begin_homing(self, object_id, homing_ang_vel):
        self.robot.cancel_nav_goals()

        if (self.robot.is_object_found(object_id)):
            return

        if isinstance(self.current_behaviour, Exploration):
            rospy.loginfo("[HOMING] Homing towards obj " + str(object_id))

            self.current_behaviour = Homing(self)
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
