from behaviour import *
from sequencer import *
from robot import *

class Behaviour:
    def __init__(self, name):
        self.name = name

    def act(self, robot, sequencer):
        robot.grid.update_grid(px, py, 'CURR')