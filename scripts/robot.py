#import follow
#from follow import Pose
#from follow import wander
from pose import Pose
import cv2
import math
import rospy

from cv_bridge import CvBridge
from areaofinterest import AreaOfInterestFinder
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

class Robot:
    def __init__(self, grid, grid_vis, aoif, laser_density, laser_angles, laser_range_max, x=0., y=0., yaw=0., sequencer = None):
        self.grid = grid
        self.grid_vis=grid_vis
        self.aoif = aoif
        self.laser_density = laser_density
        self.laser_angles = laser_angles
        self.laser_range_max = laser_range_max
        self.pose = Pose(x,y,yaw)
        self.sequencer = sequencer

        rospy.Subscriber('odom', Odometry, self.get_odom_data)
        rospy.Subscriber('scan', LaserScan, self.get_laser_data)
        rospy.Subscriber('camera/rgb/image_raw', Image, self.get_image_data)
            
    def get_odom_data(self, msg):
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(quarternion)
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y

        self.grid.update_grid(px, py, flag='CURR')
        self.pose.update_pose(px, py, yaw)

        # build contours here
        self.grid_vis.update_plot()
        self.aoif.get_grid_contours()

    def get_laser_data(self, msg):
        laser_distances = [msg.ranges[i] for i in self.laser_angles]

        for angle, dist in zip(self.laser_angles, laser_distances):
            if math.isinf(dist):  # if laser reads inf distance, clip to the laser's actual max range
                dist = self.laser_range_max

            plot_points = self.pose.plot_points_from_laser(angle, dist, self.laser_density)  # convert to a list of scanned points
            for plot_point in plot_points:
                self.grid.update_grid(plot_point[0], plot_point[1], flag='NO_OBJ')  # update the grid at each point

        # build contours here
        self.grid_vis.update_plot()
        self.aoif.get_grid_contours()

    def get_image_data(self, msg):
        # TODO
        i = 1
