from robot import Robot
import cv2
import numpy as np
from behaviours import *

class StatusWindow:
    def __init__(self, robot):
        self.robot = robot
        cv2.namedWindow('Status', 1)

    def update(self):
        i = 1

        image = np.ones([400, 600, 3]) * 255
        sequencer = self.robot.sequencer
        behaviour = ['Behaviour: ' + sequencer.current_behaviour.name]
        if isinstance(sequencer.current_behaviour, Exploration):
            behaviour = behaviour + ['  Towards: (' + str(sequencer.current_behaviour.last_goal_x) + ", "
                                                    + str(sequencer.current_behaviour.last_goal_y) + ")"]
        behaviour = behaviour + ['Idle: ' + str(self.robot.idle_tracker.idle)]

        objects = ['Objects found:',
                   '  Green cuboid: ' + str(self.robot.is_object_found(0)),
                   '  Red hydrant: ' + str(self.robot.is_object_found(1)),
                   '  Blue mailbox: ' + str(self.robot.is_object_found(2)),
                   '  White cube: ' + str(self.robot.is_object_found(3))]

        offset = 30
        x, y = 10, 30
        for idx, lbl in enumerate(behaviour + objects):
            cv2.putText(image, str(lbl), (x, y + offset * idx), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

        cv2.imshow("Status", image)
        cv2.waitKey(3)