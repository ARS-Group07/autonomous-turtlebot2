#!/usr/bin/env python2.7
#
import rospy
from pose import Pose

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

class Wanderer:
    """
    Basic random wandering and avoidance behaviours for the robot while it determines it's location.

    Attributes
    ----------
    laser_angles: list:
        which laser beams to sample in object detection
    state: str:
        whether to move forwards, reverse, or turn, based on sensory inputs
    obstacle_detected: bool:
        true when an object is within 30cm of the front of the robot
    pose: Pose:
        x, y and theta of the robot, mostly used for turning around; set before turning
    desired_pose: Pose:
        the desired pose to turn towards during avoidance
    """

    def __init__(self, laser_angles):
        self.laser_angles = laser_angles  # the horizontal fov of the camera
        self.state = 'MOVE'
        self.obstacle_detected = False
        self.pose = Pose()
        self.desired_pose = Pose()

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('odom', Odometry, self.get_odom_data)
        self.laser_subscriber = rospy.Subscriber('scan', LaserScan, self.get_laser_data)

    def move(self):
        # just move forwards at the current trajectory
        if self.state == 'MOVE':
            self.pub.publish(self.twist_msg(0.2, 0.))

        # if an obstacle's been detected, reverse until it's no longer detected, then start turning behaviour
        elif self.state == 'REVERSE':
            if self.obstacle_detected:
                self.pub.publish(self.twist_msg(-0.2, 0.))
            else:
                # so object is now out of sight
                random_yaw = self.pose.get_random_yaw()
                self.desired_pose = Pose(self.pose.px, self.pose.py, random_yaw)
                self.state = 'TURN'

        # turn towards the randomly chosen yaw, stop once reached it, and start moving forwards
        elif self.state == 'TURN':
            if self.pose.ang_dist(self.desired_pose) > 0.1:
                self.pub.publish(self.twist_msg(0., 0.3))
            else:
                self.state = 'MOVE'

    def get_odom_data(self, msg):
        """ Get current odometry data and update the Pose. """
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(quarternion)
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y

        self.pose = Pose(px, py, yaw)

    def get_laser_data(self, msg):
        """ Detect any close obstacles and trigger the robot to reverse if encountered. """
        laser_distances = [msg.ranges[i] for i in self.laser_angles]
        min_dist = min(laser_distances)

        # if obstacle detected within 30cm, stop and turn to a random yaw
        if min_dist < 0.3:
            self.obstacle_detected = True
            self.state = 'REVERSE'
        else:
            self.obstacle_detected = False

    @staticmethod
    def twist_msg(lin_vel=0., ang_vel=0.):
        """ Creates a Twist object to be published, with only linear and angular velocity values set. """
        msg = Twist()
        msg.linear.x = lin_vel
        msg.linear.y = 0.
        msg.linear.z = 0.
        msg.angular.x = 0.
        msg.angular.y = 0.
        msg.angular.z = ang_vel

        return msg

    def unsubscribe(self):
        self.odom_subscriber.unregister()  # unsubscribe from the topic here to avoid congestion
        self.laser_subscriber.unregister()  # unsubscribe from the topic here to avoid congestion
