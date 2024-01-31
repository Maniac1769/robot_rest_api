# Robot REST API

## Description

This repository describes how to use REST API to display navigation status of the Turtlebot3 which is published on the /move_base/status topic.

TurtleBot3 is a small, affordable, programmable, ROS-based mobile robot for use in education, research, hobby, and product prototyping. The goal of TurtleBot3 is to dramatically reduce the size of the platform and lower the price without having to sacrifice its functionality and quality, while at the same time offering expandability. The TurtleBot3 can be customized into various ways depending on how you reconstruct the mechanical parts and use optional parts such as the computer and sensor.

For this task, the Flask REST API was used. Flask is a web framework that allows developers to build lightweight web applications quickly and easily with Flask Libraries. 

Flask-RESTful is an extension for Flask that adds support for quickly building REST APIs. It is a lightweight abstraction that works with your existing ORM/libraries. Flask-RESTful encourages best practices with minimal setup.

- The REST API server serves the GET endpoint /api/robot/status locally on port 7201.
- The API server communicates with ROS and pulls the status information from the /move_base/status topic.
- When called, the REST API endpoint provides a response containing the status and text fields from the /move_base/status topic. If the topic is not available, the REST API calls an error out with a 400 code and a Bad request message.
1. Status: 200 OK
```
Example 1
{
“status”: 3
“text”: “Goal reached.”
}
Example 2
{
“status”: 1
“text”: “This goal has been accepted by the
simple action server”
}
```
2. Status: 400 Bad request
```
Example
{
“message”: “Invalid status value”
}
```
- The test python script polls the above REST API endpoint at a fixed frequency of 1Hz and prints the response to the console.

## Prerequisites

- Linux distro
- ROS
- Flask
- Flask-RESTful
- requests

- Install the prerequisites and packages before proceeding. This code has been run on Ubuntu 20.04 LTS ROS Noetic.

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

```
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
- Open the gazebo world using above command.

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

## Run the REST API

```
$ git clone https://github.com/Ansh-Saraiya/robot_rest_api.git
$ cd ..
$ catkin_make
$ roslaunch robot_rest_api rest_server.launch
```

## Run the test script

```
$ cd robot_rest_api
$ python src/rest_test.py
```

## References

- [Turtlebot3 SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#slam)
- [Turtlebot3 Navigation Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/)