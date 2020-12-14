from pose import Pose
import math
import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from move_base_msgs.msg import MoveBaseGoal
from ars.msg import Detection


class Robot:
    def __init__(self, grid, grid_resolution, grid_vis, aoif, laser_angles, laser_range_max, nav_client, map_arr,
                 x=0., y=0., yaw=0., sequencer=None):
        self.grid = grid
        self.grid_resolution = grid_resolution
        self.grid_vis = grid_vis
        self.aoif = aoif
        self.laser_angles = laser_angles
        self.laser_range_max = laser_range_max
        self.nav_client = nav_client
        self.pose = Pose(x, y, yaw)
        self.sequencer = sequencer
        self.map_arr = map_arr
        self.idle_tracker = IdleTracker(self, 0.0001, 30)

        # ========== HOMING & OBJECT DETECTION ==========
        # Mappings:
        # 0 - Green cuboid
        # 1 - Red fire hydrant
        # 2 - Blue mailbox
        # 3 - White, numbered (5) cube
        self.objects_found = {0: False, 1: False, 2: False, 3: False}  # Whether it's been found (homed towards)
        self.seen_store = SeenObjectStore()

        self.last_laser_msg = None
        self.homing_vel = 0

        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.get_amcl_data)
        rospy.Subscriber('scan', LaserScan, self.get_laser_data)

        # Object detection
        rospy.Subscriber('detection_green', Detection, self.object_detected_callback)
        rospy.Subscriber('detection_red', Detection, self.object_detected_callback)
        rospy.Subscriber('detection_blue', Detection, self.object_detected_callback)
        rospy.Subscriber('detection_text', Detection, self.object_detected_callback)

    def get_amcl_data(self, msg):
        """ Gets predicted position data from the adaptive Monte Carlo module and uses it for the grids, etc. """
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(quarternion)
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y

        self.pose.update_pose(px, py, yaw)
        self.grid.update_grid(px, py, flag='CURR')
        # self.grid_vis.update_plot()
        if self.grid.is_fully_explored():
            rospy.loginfo('Map fully explored, resetting ...')
            self.grid.reset_grid(self.map_arr)

    def get_laser_data(self, msg):
        self.last_laser_msg = msg

        laser_distances = [msg.ranges[i] for i in self.laser_angles]
        for angle, dist in zip(self.laser_angles, laser_distances):
            if math.isinf(dist):  # if laser reads inf distance, clip to the laser's actual max range
                dist = self.laser_range_max

            plot_points = self.pose.plot_points_from_laser(angle, dist,
                                                           self.grid_resolution)  # convert to a list of scanned points
            for plot_point in plot_points:
                self.grid.update_grid(plot_point[0], plot_point[1], flag='NO_OBJ')  # update the grid at each point

        # build contours here, update best contour cx, cy
        self.aoif.get_grid_contours(self.pose.px, self.pose.py)

        # Update the current position for the robot within the idle tracker since AMCL only sends messages
        # when the robot moves
        self.idle_tracker.track(self.pose)

    def object_detected_callback(self, msg):
        # Can create strange behaviour with the averages if it's under the mailbox
        if msg.id == 2:
            # Special case for the blue mailbox - only update when sufficiently distanced from it
            times_seen_mailbox = self.seen_store.times_seen[2]
            if times_seen_mailbox == 0:
                # Hasn't seen the mailbox yet so we have no reliable way of gauging how far it is
                self.seen_store.on_seen(msg.id, msg.x, msg.y)
            else:
                # We know where the mailbox is
                mailbox_pos = self.seen_store.get_average_location(2)
                dist_to_mailbox = math.sqrt((mailbox_pos[0] - msg.x) ** 2 + (mailbox_pos[1] - msg.y) ** 2)
                rospy.loginfo("DIstance to mailbox: " + str(dist_to_mailbox))
                if dist_to_mailbox > 1.75:
                    # Only update the position of the mailbox if at least 1.75 away (when underneath it starts acting
                    # strange)
                    self.seen_store.on_seen(msg.id, msg.x, msg.y)
        else:
            self.seen_store.on_seen(msg.id, msg.x, msg.y)

        self.sequencer.try_to_home(msg)

    # Get how many times an object has been detected
    def get_times_seen(self, object_id):
        return self.seen_store.times_seen[object_id]

    # Find the next object to route to if an object was seen while homing towards another object
    # Two return value cases:
    # a) (-1, None):                      We haven't seen an object that we've not already found
    # b) (object_id, [x_pos, y_pos]): We've seen an object that we've not already found
    def get_seen_unfound_object_position(self):
        for object_id in range(0, 4):
            if not self.is_object_found(object_id):
                if self.get_times_seen(object_id) > 0:
                    return object_id, self.seen_store.get_average_location(object_id)

        return -1, None  # IF THERE IS NO SEEN, UNFOUND OBJECT

    # Check whether an object has been 'found' (this means we've seen it and homed towards it)
    def is_object_found(self, object_id):
        return self.objects_found.get(object_id)

    # Set an object as 'found' (this means we've seen it and homed towards it)
    def set_object_found(self, object_id):
        if not self.is_object_found(object_id):
            print("HOMED TOWARDS AND FOUND OBJECT " + str(object_id))

        self.objects_found[object_id] = True

    def send_nav_goal(self, px, py, yaw=-1.0):
        self.cancel_nav_goals()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = px
        goal.target_pose.pose.position.y = py
        if yaw == -1.0:
            goal.target_pose.pose.orientation.w = 1.0
        else:
            euler = (0, 0, yaw)
            quaternion = quaternion_from_euler(*euler)
            goal.target_pose.pose.orientation.x = quaternion[0]
            goal.target_pose.pose.orientation.y = quaternion[1]
            goal.target_pose.pose.orientation.z = quaternion[2]
            goal.target_pose.pose.orientation.w = quaternion[3]
        self.nav_client.send_goal(goal)
        rospy.loginfo("Sent goal (" + str(goal.target_pose.pose.position.x) + ", " + str(
            goal.target_pose.pose.position.y) + "). Now waiting")
        _ = self.nav_client.wait_for_result(rospy.Duration(2))

    def cancel_nav_goals(self):
        self.nav_client.cancel_all_goals()


class IdleTracker:
    # idle_threshold is the maximum euclidean distance a robot can travel before it is no longer idle
    # poses_stored is how many of the last x poses to store when considering if the robot is idle
    def __init__(self, robot, idle_threshold, poses_stored):
        self.robot = robot
        self.idle_threshold = idle_threshold
        self.poses_stored = poses_stored
        self.poses = []  # poses[poses_stored - 1] is the latest pose
        self.idle = False

    def track(self, pose):
        if len(self.poses) == self.poses_stored:
            self.poses.pop(0)

        self.poses.append(Pose(pose.px, pose.py, pose.yaw))
        self.update_idle()

    def update_idle(self):
        if len(self.poses) != self.poses_stored:  # Insufficient data to determine whether idle
            self.idle = False
            return

        cumulative_dist = 0.
        for i in range(0, self.poses_stored - 3):
            pose_i = self.poses[i]
            pose_j = self.poses[i + 1]
            cumulative_dist = cumulative_dist + pose_i.dist(pose_j)

        self.idle = cumulative_dist < self.idle_threshold

    def flush(self):
        self.poses = []
        self.idle = False


# Stores where objects have been seen so we can easily come back to them when exploring
class SeenObjectStore:
    def __init__(self):
        self.times_seen = {0: 0, 1: 0, 2: 0, 3: 0}  # How many times it's been seen / detected

        self.unseen = 0  # The starting coordinate(s) for an unseen object
        self.positions = {0: [self.unseen, self.unseen], 1: [self.unseen, self.unseen],
                          2: [self.unseen, self.unseen], 3: [self.unseen, self.unseen]}

    def on_seen(self, object_id, x, y):
        times_seen = self.times_seen[object_id] + 1
        self.times_seen[object_id] = times_seen

        avg = self.get_average_location(object_id)
        # Times the average position for this object by the amount of times it has been seen
        avg[0] *= times_seen - 1
        avg[1] *= times_seen - 1
        # Add the coordinates where we've just seen the object
        avg[0] += x
        avg[1] += y
        # Calculate the new average
        avg[0] /= times_seen
        avg[1] /= times_seen

        # Store this new average
        self.positions[object_id] = avg

    def get_average_location(self, object_id):
        avg_position = self.positions[object_id]
        return [avg_position[0], avg_position[1]]
