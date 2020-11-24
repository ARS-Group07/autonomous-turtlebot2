from behaviour import *
from sequencer import *
from pose import Pose

class Behaviour:
    def __init__(self, name):
        self.name = name

    def act(self, robot, sequencer):
        print("Error: child class should override this")