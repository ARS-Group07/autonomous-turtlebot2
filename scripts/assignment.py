#!/usr/bin/env python2.7
#
import rospy
from robot import Robot
import sequencer
import math
import cv2
import matplotlib.pyplot as plt
import numpy as np
from grids import Grid, GridVisualiser
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import CameraInfo, LaserScan
from matplotlib.animation import FuncAnimation

def get_fov(msg):
    """ Loads CameraInfo message and returns horizontal field of view angle. """
    focal_length = msg.K[0]
    w = msg.width
    rospy.loginfo('Got focal length x: ' + str(focal_length))

    # formula for getting horizontal field of view angle (rad) from focal length
    fov = 2 * math.atan2(w, (2 * focal_length))

    # return field of view in degrees
    return fov * 180 / math.pi

def create_map_array(map_data, map_meta):
    """ On startup, get obstacle data from occupancy map and fill in the walls on our grid. """
    map_width = map_meta.width
    map_height = map_meta.height
    map_res = map_meta.resolution

    grid_size = 19.
    grid_res = 0.125
    grid_eff_size = int(grid_size / grid_res)

    np_map = np.reshape(np.array(map_data), [map_width, map_height])  # convert the input map to np array
    np_map = np_map[:380, 4:]  # crop to make our grid fit the map

    obstacle_map = np.where(np_map > 65, 255, 0).astype('float32')  # mask the array where there are obstacles detected
    obstacle_map = cv2.resize(obstacle_map, dsize=(grid_eff_size, grid_eff_size), interpolation=cv2.INTER_AREA)
    obstacle_map = np.where(obstacle_map > 25, -1., -0.25)  # finally assign obstacles to -1., all else to 0.5

    return obstacle_map.ravel()

if __name__ == '__main__':
    try:
        rospy.init_node('assignment_node', anonymous=True)

# MAP START
        occupancy_map = rospy.wait_for_message('/map', OccupancyGrid, timeout=15).data
        map_metadata = rospy.wait_for_message('/map_metadata', MapMetaData, timeout=15)
        rospy.loginfo('occupancy map type: ' + str(type(occupancy_map)))
        rospy.loginfo('occupancy map shape: ' + str(len(occupancy_map)))
        rospy.loginfo('occupancy map metadata: \nheight: ' + str(map_metadata.height) +
                    '\nwidth: ' + str(map_metadata.width) +
                    '\nresolution: ' + str(map_metadata.resolution))

        camera_metadata = rospy.wait_for_message('camera/rgb/camera_info', CameraInfo, timeout=15)
        laser_range_max = rospy.wait_for_message('scan', LaserScan, timeout=15).range_max  # max distance that laser detects
        fov = get_fov(camera_metadata)  # the field of view of the camera

        map_arr = create_map_array(occupancy_map, map_metadata)

        grid = Grid(map_arr=map_arr)
        grid_vis = GridVisualiser(grid)

        animate = FuncAnimation(grid_vis.fig, grid_vis.plot_grid, init_func=grid_vis.setup_frame)
        rospy.loginfo("Showing graph")
        plt.show(block=False)
        rospy.loginfo("Showed graph")
# MAP END

        theRobot = Robot(fov=fov, grid=grid)
        theRobot.sequencer = sequencer.Sequencer()
        theRobot.sequencer.sequence(theRobot)
        
    except rospy.ROSInterruptException:
        rospy.loginfo('ROSInterruptException encountered at %s' % rospy.get_time())
