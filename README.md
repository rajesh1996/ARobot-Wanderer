## ARobot Wanderer - Objects Collecting bot
[![Build Status](https://travis-ci.org/rajesh1996/wanderer-bot.svg?branch=master)](https://travis-ci.org/rajesh1996/wanderer-bot)
[![Coverage Status](https://coveralls.io/repos/github/rajesh1996/wanderer-bot/badge.svg?branch=master)](https://coveralls.io/github/rajesh1996/wanderer-bot?branch=master)
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
7. A-Star (Path planning)

## Agile Iterative Process Log sheet

[Agile log sheet](https://docs.google.com/spreadsheets/d/1dLMGk8zPM-85imcmCIn-f_uHfrWFpCoLMeSDqo5d0ug/edit?usp=sharing)

## Sprint Backlog

[Sprint sheet](https://docs.google.com/document/d/1nw3doTetBTEVzwYsUaO5rmn6OKx01XztPgxk0OSMTJQ/edit?usp=sharing)

## Presentation

[Slides](https://docs.google.com/presentation/d/1UMUzmukO2oE_W5D7BSV9Qj-iUwiHM7VWbbpau8O652g/edit?usp=sharing)
[Presentation](https://drive.google.com/file/d/1l7O3Kj6YJjlkRNp915RV9DtYmy1UTr3v/view)

[Video presentation](https://drive.google.com/file/d/1l7O3Kj6YJjlkRNp915RV9DtYmy1UTr3v/view)

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
* Open a new terminal and run the node to perform the collection task
```
cd catkin_ws
source ./deve/setup.bash
rosrun wanderer-bot book
```
<img src="/results/gaz.png"/>
<img src="/results/artag.png"/ width="350" height="250">



## Setup
1. Unzip the `artag.zip` folder and copy the contents into the `$HOME/.gazebo/mmodels` folder in your machine. 

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
**Record bag while Mapping and Collecting**

<br> to record Collecting in a bag file, pass a record:=true argument as shown below (Open a new Terminal)

```
source devel/setup.bash
roslaunch wanderer-bot base.launch record:=true
```

**View Log Levels**
To view Log levels using rqt console and rqt logger level
Open a new Terminal
```
rosrun rqt_console rqt_console
```
Open a new Terminal
```
rosrun rqt_logger_level rqt_logger_level
```

**Run Bag Files**
Go to the directory consisting bag file(Open a new Terminal)
You can download the pre recoeded bag in this [link](https://drive.google.com/drive/folders/1JxEAqz9UTEJX0XFYxD497ZBU5edJoSj2?usp=sharing) and play it
else run the demo yourself and follow the below instructions to play it.
Run roscore
```
roscore
```
Open a new terminal
```
cd ~/catkin_ws/
source devel/setup.bash
cd ~/catkin_ws/src/wanderer-bot/results/
rosbag play collectrecording.bag 
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
* https://dev.to/jansonsa/a-star-a-path-finding-c-4a4h

## Bugs
* The bot sometimes might not find a way to navigate through dynamic obstacles








