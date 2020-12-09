import rospy
from behaviours import Exploration, Homing
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
            self.current_behaviour.act(robot, self)

            if self.cycles % 10 == 0:
                self.status_window.update(self.cycles)

            rate.sleep()

    # Call of this function may come from various threads (i.e. topics from other nodes)
    def try_to_home(self, detection_msg):
        if self.robot.is_object_found(detection_msg.id):
            return

        if isinstance(self.current_behaviour, Exploration):
            self.robot.cancel_nav_goals()
            rospy.loginfo("[HOMING] Homing towards obj " + str(detection_msg.id))

            self.current_behaviour = Homing(self, self.robot.laser_angles)
            self.current_behaviour.set_target(self.robot, detection_msg)
        elif isinstance(self.current_behaviour, Homing):
            # Only update the angular velocity if this function call is for the same object type we've been homing
            # towards
            if self.current_behaviour.current_object_type == detection_msg.id:
                self.current_behaviour.set_target(self.robot, detection_msg)  # Just in case anything has changed

    def finished_homing(self):
        self.current_behaviour = Exploration()
