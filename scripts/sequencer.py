#
import robot
import rospy
import thread

class Sequencer:
    def __init__(self, robot):
        self.robot = robot
        self.robot.state = "wander"

        #thread.start_new_thread( self.sequence ,() )
        #yo yo yo yo

    def sequence(self):
        while not rospy.is_shutdown():

            rate = rospy.Rate(25)
            rospy.loginfo('state: ' + self.robot.state)
            if(self.robot.flag_obstacle_front is False and self.robot.flag_obstacle_right is False):
                if(self.robot.state is "follow_wall"):
                    self.robot.state = "turn_right"
                if(self.robot.state is "turn_right"):
                    self.robot.turn_right()
                else:
                   # self.robot.state = "wander"   
                    self.robot.move_forward()
                    self.robot.state = "wander"
            elif(self.robot.flag_obstacle_front is True):
                
                self.robot.turn_left()
            else:
                self.robot.state = "follow_wall"
                self.robot.move_forward()

            rate.sleep()
            