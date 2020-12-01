## ARobot Wanderer - A collecing bot

## Overview
The ARoBot Wanderer is a simulation of an object collecting turtlebot in gazebo that identifies and collects the object’s from different rooms present in the warehouse and transfers them to the desired location. The robot tasks and its environment mimics a warehouse automation setup in which there is no human intervention in collecting the objects. This application can also be extended to other robots, environments, and different object identification tasks. The bot uses the G mapping (SLAM) and RRT exploration package to map its environment and navigate to certain locations based on the data received on the LIDAR sensor.

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
8. RRT exploration (Exploration strategy)

## Build Instructions
```
cd catkin_ws/src
git clone https://github.com/rajesh1996/wanderer-bot.git
cd ..
catkin_make
```




