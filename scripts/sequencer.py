#
import robot
import rospy
import thread
import behaviour
import exploration

class Sequencer:
    def __init__(self):
        self.robot = robot

        # lower key value -> higher priority
        self.hierarchy = {1: exploration.Exploration()}

        self.current_behaviour_idx = 1
        self.current_behaviour = self.hierarchy.get(self.current_behaviour_idx)

    def sequence(self, robot):
        rate = rospy.Rate(25)

        while not rospy.is_shutdown():
            #rospy.loginfo("Behaviour: " + self.current_behaviour.name)
            self.current_behaviour.act(robot, self)
            rate.sleep()

    def at_top_hierarcgrid_resolutionhy(self):
        return self.current_behaviour_idx == 1

    def at_bottom_hierarchy(self):
        return self.current_behaviour_idx == len(self.hierarchy) - 1

    def ascend_behaviour(self):
        next_idx = self.current_behaviour_idx - 1

        if (next_idx < 1):
            rospy.loginfo("Tried to ascend hierarchy but already at the top")
        else:
            self.current_behaviour_idx = next_idx
            self.current_behaviour = self.hierarchy.get(self.current_behaviour_idx)
            rospy.loginfo("Ascended behaviour. New: " + self.current_behaviour.name)

    def descend_behaviour(self):
        next_idx = self.current_behaviour_idx + 1

        if (next_idx >= len(self.hierarchy) - 1):
            rospy.loginfo("Tried to descend hierarchy but already at the bottom")
        else:
            self.current_behaviour_idx = next_idx
            self.current_behaviour = self.hierarchy.get(self.current_behaviour_idx)
            rospy.loginfo("Descended behaviour. New: " + self.current_behaviour.name)
