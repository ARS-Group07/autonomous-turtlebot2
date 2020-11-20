#!/usr/bin/env python2.7
#
import rospy
import robot
import sequencer

if __name__ == '__main__':
    theRobot = robot.Robot()
    try:
        rospy.init_node('assignment_node', anonymous=True)

        theRobot.sequencer = sequencer.Sequencer()
        theRobot.sequencer.sequence(robot)
        
    except rospy.ROSInterruptException:
        rospy.loginfo('ROSInterruptException encountered at %s' % rospy.get_time())
