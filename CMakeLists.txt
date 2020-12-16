cmake_minimum_required(VERSION 3.0.2)
project(wanderer-bot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_core 
  move_base_msgs
  base_local_planner
  roscpp
  std_msgs
  tf
  rostest
  roscpp
  rospy
  message_generation
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wanderer-bot
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs nav_core
#  DEPENDS system_lib
)


## To run tests, add dependencies for gtest and rostest
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  ##find_package(gtest)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")

  catkin_add_gtest(testGrid test/testOccupancyGrid.cpp src/OccupancyGrid.cpp)
  target_link_libraries(testGrid ${catkin_LIBRARIES})

  catkin_add_gtest(testcollect test/testRandomizer.cpp test/testSpawnCollect.cpp src/Randomizer.cpp src/SpawnCollect.cpp)
  target_link_libraries(testcollect ${catkin_LIBRARIES})

  add_rostest_gtest(testPlanner launch/testAstarPlanner.launch test/main.cpp test/testAstarPlanner.cpp src/OccupancyGrid.cpp src/AstarPlanner.cpp)
  target_link_libraries(testPlanner ${catkin_LIBRARIES})
  add_dependencies(testPlanner pathPlanner ${catkin_EXPORTED_TARGETS})

 add_rostest_gtest(testNavigator launch/testNavigator.launch test/testNavigatebot.cpp src/Navigatebot.cpp)
  target_link_libraries(testNavigator ${catkin_LIBRARIES})
  add_dependencies(testNavigator astar_move ${catkin_EXPORTED_TARGETS})
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})



add_library(pathPlanner src/OccupancyGrid.cpp src/AstarPlanner.cpp)


add_executable(book src/Randomizer.cpp src/SpawnCollect.cpp src/Collector.cpp)
target_link_libraries(book ${catkin_LIBRARIES})

add_executable(astar_move src/main.cpp src/Navigatebot.cpp)
add_dependencies(astar_move ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(astar_move ${catkin_LIBRARIES})
