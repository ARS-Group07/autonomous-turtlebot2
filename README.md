# ARS Assignment

## Directory
### Setup
In order to be able to launch the project the store/cfg and store/data folders must be populated as the contents of them are too large to be shared over git.

These files must be put in the directories specified in the tree in order for the paths to properly be resolved in detection_paths.py.

### Directory Tree
```bash
├── CMakeLists.txt
├── launch
│   ├── launch.launch
│   └── turtlebot3_training.launch
├── package.xml
├── scripts
│   ├── areaofinterest.py
│   ├── assignment.py
│   ├── behaviours.py
│   ├── detection
│   │   ├── areaofinterest.pyc
│   │   ├── behaviours.pyc
│   │   ├── bluegreen.py
│   │   ├── depth.py
│   │   ├── detection_paths.py
│   │   ├── grids.pyc
│   │   ├── hydrant.py
│   │   ├── localise.pyc
│   │   ├── messagehelper.pyc
│   │   ├── pose.pyc
│   │   ├── robot.pyc
│   │   ├── sequencer.pyc
│   │   ├── status.pyc
│   │   ├── textdetect.py
│   │   └── text.py
│   ├── grids.py
│   ├── localise.py
│   ├── messagehelper.py
│   ├── pose.py
│   ├── robot.py
│   ├── sequencer.py
│   └── status.py
├── src
│   ├── clean_map.pgm
│   └── clean_map.yaml
├── store
│   ├── cfg
│   │   ├── yolo9000.cfg
│   │   ├── yolov3.cfg
│   │   ├── yolov3-openimages.cfg
│   │   ├── yolov3-spp.cfg
│   │   ├── yolov3-tiny.cfg
│   │   └── yolov3-voc.cfg
│   └── data
│       ├── coco.names
│       ├── frozen_east_text_detection.pb
│       ├── yolov3-tiny.weights
│       └── yolov3.weights
└── worlds
    └── comp4034_maze1.world
```

## Run Instructions
### Step 1: Launch environment 
```bash
roslaunch ars turtlebot3_training.launch 
```

### Step 2: Launch RViz with map 
```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_ws/src/ars/src/clean_map.yaml
```

### Step 3: Remove ghost walls after setting 2D pose estimate in RViz
```bash
rosservice call /move_base/clear_costmaps
```
