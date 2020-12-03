#!/usr/bin/env python2.7
#
import messagehelper
import rospy
import sequencer
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from robot import Robot
from localise import Localiser, Wanderer
from areaofinterest import AreaOfInterestFinder
from geometry_msgs.msg import PoseWithCovarianceStamped
from sequencer import Sequencer
from grids import Grid, GridVisualiser
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import CameraInfo, LaserScan


def localise(laser_angles):
    # SRY 4 THE MESS BEN :'(
    #
    #
    # Localise self before continuing to remainder of program
    first_amcl_msg = rospy.wait_for_message('amcl_pose', PoseWithCovarianceStamped, timeout=5)
    rospy.loginfo('Got AMCL message, starting robot localisation ... ')

    localiser = Localiser()
    wanderer = Wanderer(laser_angles)

    r = rospy.Rate(15)
    while not localiser.localised:
        wanderer.move()
        r.sleep()
    rospy.loginfo('ROBOT LOCALISED')

    # robot now localised so kill those subscribers - no longer needed
    localiser.unsubscribe()
    wanderer.unsubscribe()
    rospy.loginfo('Killed localisation and wandering behaviours - initialising full robot functionality ... ')


if __name__ == '__main__':
    try:
        rospy.init_node('assignment_node', anonymous=True)

        # wait for all important messages to arrive from various nodes
        occupancy_map = rospy.wait_for_message('/map', OccupancyGrid, timeout=5).data
        map_metadata = rospy.wait_for_message('/map_metadata', MapMetaData, timeout=5)
        camera_metadata = rospy.wait_for_message('camera/rgb/camera_info', CameraInfo, timeout=5)
        laser_range_max = rospy.wait_for_message('scan', LaserScan, timeout=5).range_max

        # Nav Stack
        nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for nav client...")
        nav_client.wait_for_server()
        rospy.loginfo("Waited for nav client...")

        # ========== SETTINGS ==========
        grid_resolution = 0.2
        laser_density = 4  # eg sample a laser beam every 4 degrees
        fov = messagehelper.get_fov(camera_metadata)  # the field of view of the camera
        rospy.loginfo('fov: ' + str(fov))
        laser_angles = list(range(-int(fov / 2.), 0, laser_density)) + list(range(0, int(fov / 2.), laser_density))

        # Localise ourself using Monte Carlo if we're not skipping (takes a lot of time to initialise)
        skip_localisation = False
        if (not skip_localisation):
            localise(laser_angles)
        # once localised, continue to the remainder of the code ...

        # ========== grid and visualiser initialisation ==========
        map_arr = messagehelper.create_map_array(occupancy_map, map_metadata, grid_resolution)
        grid = Grid(map_arr=map_arr)

        # Instantiate and show the AOI Finder & grid visualiser
        grid_vis = GridVisualiser(grid)
        aoif = AreaOfInterestFinder(grid, scale=4)

        the_robot = Robot(grid=grid, grid_resolution = grid_resolution, grid_vis=grid_vis,
                          aoif=aoif, laser_angles = laser_angles,laser_range_max=laser_range_max,
                          nav_client=nav_client, use_amcl_localisation=(not skip_localisation))
        the_robot.sequencer = Sequencer(the_robot)
        the_robot.sequencer.sequence(the_robot)
        
    except rospy.ROSInterruptException:
        rospy.loginfo('ROSInterruptException encountered at %s' % rospy.get_time())
