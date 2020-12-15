#!/usr/bin/env python2.7
import os

# Contains the paths for all of the required files (all relative to BASE_PATH)
class Paths:
    USERNAME = os.getlogin()
    BASE_PATH = '/home/' + USERNAME + '/catkin_ws/src/ars/store/'

    CONFIG_FILE = BASE_PATH + 'cfg/yolov3.cfg'
    LABELS_FILE = BASE_PATH + 'data/coco.names'
    WEIGHTS_FILE = BASE_PATH + 'data/yolov3.weights'
    TEXT_FILE = BASE_PATH + "data/frozen_east_text_detection.pb"
