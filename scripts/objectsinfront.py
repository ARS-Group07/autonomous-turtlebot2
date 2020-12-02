class VisibleObject:
    def __init__(self, object_type, wx, wy, gx, gy):
        self.object_type = object_type # Map discrete values to each type of object here
        self.wx = wx
        self.wy = wy
        self.gx = gx
        self.gy = gy

    def is_of_interest(self):
        return self.object_type < 3 # for example

class LineOfSight:
    """ Class that stores all currently visible objects to the robot. """

    def __init__(self, grid, objects = []):
        self.grid = grid
        self.objects = objects

    def add_object(self, object):
        self.objects.append(object)

    ## TODO - MAP STUFF HERE - DECIDE WHETHER THIS HAS ALREADY BEEN DETECTED (ALTHOUGH, THIS IS RESOLUTUION DEPENDENT)

class LineOfSightGenerator:
    """ Factory class that generates a LineOfSight object given instances of sensory data. """

    def __init__(self, the_robot):
        self.the_robot = the_robot

    def create_and_populate(self, last_laser_msg, last_image_msg):
        if (last_image_msg == None or last_laser_msg == None): # Still waiting on one of the messages to come through
            return LineOfSight(self.the_robot.grid, [])

        # CEM: TODO - detect objects and stuff here
        objects = []

        return LineOfSight(self.the_robot.grid, objects)
