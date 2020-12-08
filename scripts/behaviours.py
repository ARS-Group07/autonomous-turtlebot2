import random
import rospy
from geometry_msgs.msg import Twist


class Behaviour:
    def __init__(self, name):
        self.name = name

    def act(self, robot, sequencer):
        print("Error: child class should override this")


class Exploration(Behaviour):
    def __init__(self):
        Behaviour.__init__(self, "Exploration")
        self.prev_cx = 0.
        self.prev_cy = 0.

    def act(self, robot, sequencer):
        aoif = robot.aoif

        # only update goal once every 2 seconds
        if sequencer.i % (sequencer.sequence_hz * 2) == 0:
            sequencer.i = 0
            wx, wy = robot.grid.to_world(aoif.closest_cx / aoif.scale, aoif.closest_cy / aoif.scale)

            if self.prev_cx == aoif.closest_cx and self.prev_cy == aoif.closest_cy:
                rospy.loginfo('contour not moved: randomizing position slightly ')
                wx += round(random.random(), 1) - 0.5
                wy += round(random.random(), 1) - 0.5

            self.prev_cx = aoif.closest_cx
            self.prev_cy = aoif.closest_cy
            robot.send_nav_goal(wx, -wy - 0.5)  # hack because y always slightly shifted; probably map is a bit off


class Homing(Behaviour):
    # Static variable
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def __init__(self, sequencer, laser_angles):
        Behaviour.__init__(self, "Homing")

        self.sequencer = sequencer
        self.laser_angles = laser_angles
        self.current_object_type = -1
        self.ang_vel = 0

    def act(self, robot, sequencer):
        if robot.last_laser_msg is None:
            return

        # First check if we're sufficiently close to an object, in which case we'll either ignore homing /
        laser_distances = [robot.last_laser_msg.ranges[i] for i in self.laser_angles]
        min_dist = min(laser_distances)

        twist = Twist()
        if min_dist < 1.0:
            twist.linear.x = 0.1
            twist.angular.z = self.ang_vel
        else:
            twist.linear.x = 0.
            twist.angular.z = 0.
            robot.set_object_found(self.current_object_type)
            self.sequencer.finished_homing()

        Homing.velocity_publisher.publish(twist)
