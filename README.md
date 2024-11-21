## 🤖 Robotics Lab Homework2

This repository contains the three ROS2 packages required for the homework 2 of the course. In the following the instructions to run the packages correctly.

## 📦 Build and Source

Once you cloned the repository, build the packages.

```bash
colcon build
```

Then source 

```bash
source install/setup.bash 
```


## 🔥 Visualize the manipulator in Gazebo 

```bash

ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller"
```



## 🤙 Send the effort command to the manipulator

In another terminal run

```bash

ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=effort
```



