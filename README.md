# Autonomous robotics with TurtleBot2 

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
