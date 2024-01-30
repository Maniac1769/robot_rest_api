# Robot REST API

## Description

- This repository describes how to use REST API to display navigation status of the Turtlebot3 which is published on the /move_base/status topic.

## Prerequisites

- Linux distro
- ROS

- This code has been run on Ubuntu 20.04 LTS ROS Noetic.

## Install Dependent ROS Packages

```
$ sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
  ```
## Install Turtlebot3 Packages

```
$ sudo apt install ros-noetic-dynamixel-sdk
$ sudo apt install ros-noetic-turtlebot3-msgs
$ sudo apt install ros-noetic-turtlebot3
```

## Creating Map using SLAM

- Run ROS environment
```
$ roscore
```

### Run SLAM node
- The default SLAM method is Gmapping. Please use the proper keyword among burger, waffle, waffle_pi for the TURTLEBOT3_MODEL parameter.
```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_slam turtlebot3_slam.launch
```

- Once SLAM node is successfully up and running, TurtleBot3 will be exploring unknown area of the map using teleoperation. It is important to avoid vigorous movements such as changing the linear and angular speed too quickly. When building a map using the TurtleBot3, it is a good practice to scan every corner of the map.

- Open a new terminal and run the Teleop node 
```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### Save the map
```
$ rosrun map_server map_saver -f ~/map
```
- The -f option specifies a folder location and a file name where files to be saved. With the above command, map.pgm and map.yaml will be saved in the home folder ~/(/home/${username}).

## Navigation Simulation

- Launch the gazebo world.
```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

- Open a new terminal and run the the navigation node.
```
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```

### Estimate Initial Pose

- Click the 2D Pose Estimate button in the RViz menu.
- Click on the map where the actual robot is located and drag the large green arrow toward the direction where the robot is facing.
- Launch keyboard teleoperation node to precisely locate the robot on the map.
- Move the robot back and forth a bit to collect the surrounding environment information and narrow down the estimated location of the TurtleBot3 on the map which is displayed with tiny green arrows.
- Terminate the keyboard teleoperation node by entering Ctrl + C to the teleop node terminal in order to prevent different cmd_vel values are published from multiple nodes during Navigation.

### Set Navigation Goal

- Click the 2D Nav Goal button in the RViz menu.
- Click on the map to set the destination of the robot and drag the green arrow toward the direction where the robot will be facing.