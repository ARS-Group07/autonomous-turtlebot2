import rospy
from tf.transformations import euler_from_quaternion

class Odom:
    def __init__(self, robot):
        self.robot = robot

    def get_odom_data(self, msg):
            """ Converts to euler angles, saves the received odom data as a dictionary and appends to global odom_msgs list. """
            quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            (_, _, yaw) = euler_from_quaternion(quarternion)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

    
            self.robot.odom_data = {'x': x, 'y': y, 'yaw': yaw}



       

   