## ARobot Wanderer - Objects Collecting bot
[![Build Status](https://travis-ci.org/rajesh1996/wanderer-bot.svg?branch=master)](https://travis-ci.org/rajesh1996/wanderer-bot)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://github.com/rajesh1996/wanderer-bot/blob/master/LICENSE)

## Overview
The ARoBot Wanderer is a simulation of an object collecting turtlebot in gazebo that identifies and collects the object’s from different rooms present in the warehouse and transfers them to the desired location. The robot tasks and its environment mimics a warehouse automation setup in which there is no human intervention in collecting the objects. This application can also be extended to other robots, environments, and different object identification tasks. The bot uses the G mapping (SLAM) to map its environment and navigate to certain locations based on the data received on the LIDAR sensor.

## Authors
* Arjun Srinivasan
* Rajeshwar N S

## Dependencies
1. Ubuntu 18.04 
2. ROS Melodic (Robotics Middleware) 
3. Gazebo (3D physics Simulation environment) 
4. Robot Model: TurtleBot3
5. Rviz (Sensor visualization) 
6. G mapping (SLAM)
7. A-Star (Optimal Path planning)

## Agile Iterative Process Log sheet

[Agile log sheet](https://docs.google.com/spreadsheets/d/1dLMGk8zPM-85imcmCIn-f_uHfrWFpCoLMeSDqo5d0ug/edit?usp=sharing)

## Sprint Backlog

[Sprint sheet](https://docs.google.com/document/d/1nw3doTetBTEVzwYsUaO5rmn6OKx01XztPgxk0OSMTJQ/edit?usp=sharing)

## Presentation

[Slides](https://docs.google.com/presentation/d/1UMUzmukO2oE_W5D7BSV9Qj-iUwiHM7VWbbpau8O652g/edit?usp=sharing)

## Build Instructions

* Clone the repo and catkin_make
```
cd catkin_ws/src
git clone https://github.com/rajesh1996/wanderer-bot.git
cd ..
source devel/setup.bash
catkin_make
```
* Launch the Gazebo world file
```
roslaunch wanderer-bot base.launch
```
* Open a new terminal and run the node
```
cd catkin_ws
source ./deve/setup.bash
rosrun wanderer-bot book
```

## Generating Map for custom world file
* To build a map for a custom world file, run the following
```
roslaunch wanderer-bot map.launch 
```
* Run the turtlbot3 teleop node
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch  
```
* Run SLAM-gmapping package to generate a map using the custom world file with collect_world
```
rosrun map_server map_saver -f ~/collect_world
```

## Run ROS Tests
```
catkin_make run_tests_wanderer-bot
```

## Doxygen
* To generate doxygen file, run
```
sudo apt-get install doxygen
sudo apt install doxygen-gui
doxywizard
```

## References
* https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/
* http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin
* https://github.com/coins-lab/relaxed_astar
* http://wiki.ros.org/rrt_exploration
* https://dev.to/jansonsa/a-star-a-path-finding-c-4a4h

## Bugs
* The bot sometimes might not find a way to navigate through dynamic obstacles








