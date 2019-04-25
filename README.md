# Turtlebot Playground

Welcome to Turtlebot Playground! This is a Gazebo environment for testing out path planning methods on the Turtlebot in simulation.

Maintainer: Rosalind Shinkle <rshinkle@seas.upenn.edu>

## Prerequisites
1. You must have at least Gazebo 8, which requires at least ROS Melodic, to run this code - you will also need basic Python packages like numpy, scipy, shapely, etc. - if you don't have these packages, trying to run point navigation will fail, and the error will tell you which package to install. 
* For a proper Gazebo installation, install `ros-melodic-desktop-full` and then follow the instructions for a step-by-step Gazebo 9 install [here](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install). Finally, run a `sudo apt-get upgrade` to update all necessary libraries.

2. Clone this repository into your catkin workspace

3. Add ROS package path to your bashrc

4. Install dependencies:
```
$ sudo apt-get install ros-melodic-controller-manager ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control ros-melodic-effort-controllers ros-melodic-joystick-drivers ros-melodic-ecl ros-melodic-yocs-controllers ros-melodic-yocs-cmd-vel-mux ros-melodic-kobuki-dock-drive ros-melodic-kobuki-driver ros-melodic-depthimage-to-laserscan
```

5. Install turtlebot from source because ROS Melodic does not include Turtlebot 2 with its binaries:
```
$ git clone https://github.com/turtlebot/turtlebot
```

6. Install Kobuki pkgs (not yet released for melodic - see thread: https://github.com/turtlebot/turtlebot/issues/272)
```
$ git clone https://github.com/yujinrobot/kobuki.git
$ git clone https://github.com/yujinrobot/kobuki_msgs.git
$ git clone https://github.com/yujinrobot/kobuki_desktop.git
$ cd kobuki_desktop/
$ rm -r kobuki_qtestsuite
```

## Usage
1. Launch for empty world:
```
$ roslaunch turtlebot_playground turtlebot_kodworld.launch
```

Launch for PERCH world:
```
$ roslaunch turtlebot_playground turtlebot_perchworld.launch
```

2. For teleop, launch in a new terminal:
```
$ roslaunch turtlebot_teleop keyboard_teleop.launch
```

3. For RViz, launch in a new terminal:
```
$ roslaunch turtlebot_rviz_launchers view_robot.launch
```

4. (Optional) For shell navigation, launch in a new terminal:
```
$ roslaunch turtlebot_playground shell_navigation.launch
```

