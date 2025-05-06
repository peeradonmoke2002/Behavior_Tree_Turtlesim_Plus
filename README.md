# Behavior Tree With Turtlesim Plus
 
## Table of Contents
- [Description](#description)
- [Installation](#installation)
- [Move to center](#move-to-center)
- [Square Path](#square-path)

## Description
This package I have use BT with turtlesim to show how to use behavior tree with turtlesim.

1) The turtlesim will always check and move to center:

![Turtlesim Behavior Tree Move to Center](./images/2025-05-06%2015-14-52.gif)

2) The turtlesim will as square path and check if have obstacle it will crate new path to pass the obstacle:

comming soon!


## Installation

Step 1: Clone the repository
```bash
git clone https://github.com/peeradonmoke2002/Behavior_Tree_Turtlesim_Plus.git
```

Step 2: Install dependencies
```bash
cd Behavior_Tree_Turtlesim_Plus
rosdep install --from-paths src --ignore-src -r -y
```

Step 3: Build the package
```bash
colcon build
```

Step 4: Source the workspace
```bash
source install/setup.bash
```
Ready to use now!!


## Move to center

In this section I will show how to use behavior tree with turtlesim to move to center of turtlesim. First you need to design the behavior tree like my design below:

![Behavior Tree Move to Center](./images/Screenshot%202568-05-06%20at%2015.28.57.png)

Here is video how is look like:
![Turtlesim Behavior Tree Move to Center](./images/2025-05-06%2015-14-52.gif)


### Run the turtlesim

1) Open a new terminal and run the turtlesim node:
```bash
ros2 run turtlesim_plus turtlesim_plus_node.py
```
2) Open a new terminal and run teleop keyboard:
```bash
ros2 run turtlesim_plus_teleop turtlesim_plus_teleop
```
3) Open a new terminal and run the behavior tree Viwer
```bash
py-trees-tree-viewer
```
4) Open a new terminal and run the behavior tree:
```bash
ros2 run bt_turtlesim_plus turtle_bt_center.py
```
5) Try to move to anywhere in the turtlesim and see how the behavior tree work. 

## Square Path
Coming soon!