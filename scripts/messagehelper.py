import math
import cv2
import numpy as np

def get_fov(msg):
    """ Loads CameraInfo message and returns horizontal field of view angle. """
    focal_length = msg.K[0]
    w = msg.width

    # formula for getting horizontal field of view angle (rad) from focal length
    fov = 2 * math.atan2(w, (2 * focal_length))

    # return field of view in degrees
    return round(fov * 180 / math.pi)

def create_map_array(map_data, map_meta, grid_resolution):
    """ On startup, get obstacle data from occupancy map and fill in the walls on our grid. """
    map_width = map_meta.width
    map_height = map_meta.height

    grid_size = round(map_width * map_meta.resolution, 1)
    grid_eff_size = int(grid_size / grid_resolution)

    np_map = np.reshape(np.array(map_data), [map_height, map_width])  # convert the 1D input map to a 2D np array
    obstacle_map = np.where(np.logical_or((np_map > 0), (np_map < -0.5)), 255., 0.)  # mask obstacles and unmapped areas
    obstacle_map = cv2.resize(obstacle_map, dsize=(grid_eff_size, grid_eff_size), interpolation=cv2.INTER_AREA)
    obstacle_map = np.where(obstacle_map > 0, -1., 0.5).astype('float32')  # inaccessible areas to -1., all else to 0.5

    return obstacle_map
