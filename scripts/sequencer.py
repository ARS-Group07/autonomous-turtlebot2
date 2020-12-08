from behaviours import *


class Sequencer:
    def __init__(self, robot):
        self.robot = robot
        self.i = 0
        self.sequence_hz = 25

        self.current_behaviour = Exploration()

    def sequence(self, robot):
        rate = rospy.Rate(self.sequence_hz)

        while not rospy.is_shutdown():
            self.i += 1
            # rospy.loginfo("Behaviour: " + self.current_behaviour.name)
            self.current_behaviour.act(robot, self)
            rate.sleep()

    # Call of this function may come from various threads (i.e. topics from other nodes)
    def begin_homing(self, object_id, homing_ang_vel):
        self.robot.cancel_nav_goals()

        if self.robot.is_object_found(object_id):
            return

        if isinstance(self.current_behaviour, Exploration):
            self.current_behaviour = Homing(self, self.robot.laser_angles)
            self.current_behaviour.current_object_type = object_id
            self.current_behaviour.ang_vel = homing_ang_vel
        elif isinstance(self.current_behaviour, Homing):
            # Only update the angular velocity if this function call is for the same object type we've been homing
            # towards
            if self.current_behaviour.current_object_type == object_id:
                # TODO SOME SORT OF PRIORITY SYSTEM USING THE OBJECT ID IN CASE BOTH ARE IN THE SAME IMAGE FRAME
                self.current_behaviour.current_object_type = object_id
                self.current_behaviour.ang_vel = homing_ang_vel

    def finished_homing(self):
        self.current_behaviour = Exploration()
