# Autonomous robotics with TurtleBot2 
Our implementation of an autonomous exploring and goal seeking system using the TurtleBot2 in ROS. Utilizes the ROS navigation stack including the adaptive Monte Carlo algorithm for localisation, custom display and mapping modules and a custom-built grid map. Uses OpenCV for visual data processing as well as our novel method of efficient map exploration using unexplored area centroids. 

## Run Instructions
### Step 1: Launch environment 
```bash
roslaunch ars turtlebot3_training.launch 
```

### Step 2: Launch RViz with map 
```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_ws/src/ars/src/clean_map.yaml prefer_forward_cost_function:=5.0
```

### Step 3: Remove ghost walls after setting 2D pose estimate in RViz
```bash
rosservice call /move_base/clear_costmaps
```

### Step 4: Launch
```bash
roslaunch ars launch.launch
```
