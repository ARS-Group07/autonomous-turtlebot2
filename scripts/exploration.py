from behaviour import *
from sequencer import *
from robot import *


class Exploration(Behaviour):
    def __init__(self):
        Behaviour.__init__(self, "Exploration")

    def act(self, robot, sequencer):
        i = 1
        #print("Test.")
        #sequencer.descend_behaviour()