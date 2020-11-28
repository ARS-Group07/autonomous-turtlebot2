#!/usr/bin/env python2.7
#
import rospy
from pose import Pose

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


class Localiser:
    """ Localise the robot after initialisation by randomly wandering until variance is low. """

    def __init__(self):
        # note that amcl_pose only publishes when you actually move around
        self.amcl_pose_subscriber = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.get_confidence)
        self.localised = False

    def get_confidence(self, msg):
        # flattened  6x6 covariance matrix for x, y, z, ang_x, ang_y, ang_z from PoseWithCovarianceStamped
        covariance_mx = msg.pose.covariance

        # get variance values for x, y and theta position to determine robot confidence in positioning
        x_var = covariance_mx[0]
        y_var = covariance_mx[7]
        theta_var = covariance_mx[35]

        rospy.loginfo('Location confidence check: x var: %.4f, y var: %.4f, theta var: %.4f' % (x_var, y_var, theta_var))

        # variance = 0.01 means average of estimates squared distances from the mean estimate = 0.01m^2
        # so average distance is around 0.1m (apply sq. root)
        # assuming Gaussian distribution this is confidence margin of around +-10cm in x and y directions
        # for theta this is around +-6 degrees once converted from radians

        if x_var < 0.01 and y_var < 0.01 and theta_var < 0.01:
            rospy.loginfo('AMCL CONFIDENCE ACHIEVED! x, y and theta variances < 0.01')
            self.localised = True

    def unsubscribe(self):
        self.amcl_pose_subscriber.unregister()  # unsubscribe from the topic here to avoid congestion


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
                self.pub.publish(self.twist_msg(0., -0.3))
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
