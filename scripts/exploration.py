from behaviour import *
from sequencer import *
from robot import *
from areaofinterest import AreaOfInterestFinder

class Exploration(Behaviour):
    def __init__(self):
        Behaviour.__init__(self, "Exploration")
        self.last_goal_x = 0
        self.last_goal_y = 0

    def act(self, robot, sequencer):
        aoif = robot.aoif
        if (not aoif.largest_area == -1):
            if ( (not aoif.largest_cx == self.last_goal_x) and (not aoif.largest_cy == self.last_goal_y) ):
                self.last_goal_x = aoif.largest_cx
                self.last_goal_y = aoif.largest_cy

                # TODO: Convert gx, gy to px, py
                wx, wy = robot.grid.to_world(aoif.largest_cx / aoif.scale,
                                             aoif.largest_cy / aoif.scale)
                robot.send_nav_goal(wx, wy)