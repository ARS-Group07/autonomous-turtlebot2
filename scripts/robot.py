from pose import Pose
import math
import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, Image
from tf.transformations import euler_from_quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from objectsinfront import LineOfSight, VisibleObject, LineOfSightGenerator


class Robot:
    def __init__(self, grid, grid_resolution, grid_vis, aoif, laser_angles, laser_range_max, nav_client,
                 x=0., y=0., yaw=0., sequencer = None):
        self.grid = grid
        self.grid_resolution = grid_resolution
        self.grid_vis = grid_vis
        self.aoif = aoif
        self.laser_angles = laser_angles
        self.laser_range_max = laser_range_max
        self.nav_client = nav_client
        self.pose = Pose(x,y,yaw)
        self.sequencer = sequencer

        # Line of Sight
        self.last_laser_msg = None
        self.last_image_msg = None
        self.los_generator = LineOfSightGenerator(self)
        self.current_los = self.los_generator.create_and_populate(None, None)

        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.get_amcl_data)
        rospy.Subscriber('scan', LaserScan, self.get_laser_data)
        rospy.Subscriber('camera/rgb/image_raw', Image, self.get_image_data)

    # odom is gone
    def get_amcl_data(self, msg):
        """ Gets predicted position data from the adaptive Monte Carlo module and uses it for the grids, etc. """
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(quarternion)
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y

        self.pose.update_pose(px, py, yaw)
        self.grid.update_grid(px, py, flag='CURR')
        self.grid_vis.update_plot()

    def get_laser_data(self, msg):
        self.last_laser_msg = msg

        laser_distances = [msg.ranges[i] for i in self.laser_angles]
        for angle, dist in zip(self.laser_angles, laser_distances):
            if math.isinf(dist):  # if laser reads inf distance, clip to the laser's actual max range
                dist = self.laser_range_max

            plot_points = self.pose.plot_points_from_laser(angle, dist, self.grid_resolution)  # convert to a list of scanned points
            for plot_point in plot_points:
                self.grid.update_grid(plot_point[0], plot_point[1], flag='NO_OBJ')  # update the grid at each point

        # build contours here
        self.aoif.get_grid_contours()

    def get_image_data(self, msg):
        self.last_image_msg = msg

        # Find the objects within the Line of Sight here (can do this here and/or in the laser callback, but performance)
        self.current_los = self.los_generator.create_and_populate(self.last_image_msg, self.last_image_msg)

    def send_nav_goal(self, px, py):
        self.cancel_nav_goals()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = px
        goal.target_pose.pose.position.y = py
        goal.target_pose.pose.orientation.w = 1.0
        self.nav_client.send_goal(goal)
        rospy.loginfo("Sent goal (" + str(goal.target_pose.pose.position.x) + ", " + str(
            goal.target_pose.pose.position.y) + "). Now waiting")
        wait = self.nav_client.wait_for_result()

    def cancel_nav_goals(self):
        self.nav_client.cancel_all_goals()
