/*****************************************************************************************
 Copyright (C) 2020 Arjun Srinivasan Ambalam,Rajeshwar N.S
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
   * Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
******************************************************************************************/

/**
 *  @copyright MIT License 2020 Arjun Srinivasan Ambalam,Rajeshwar N.S
 *  @file    testAstarPlanner.cpp
 *  @author  Arjun Srinivasan Ambalam
 *  @author  Rajeshwar N.S
 *  @date    12/07/2020
 *  @version 1.0
 *
 *  @brief Final Project - wanderer-bot (Search and collect objects)
 *
 *  @section DESCRIPTION
 *  Test cases for pathPlanner Algorithm
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <memory>
#include <set>
#include <numeric>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <iterator>
#include <algorithm>
#include <string>
#include <vector>
#include <sstream>
#include <map>
#include <limits>
#include "../include/AstarPlanner.hpp"
#include "../include/OccupancyGrid.hpp"

/**
 * @def TEST(TestPathPlanner, testNodeHandle)
 * @brief To check if the robot node is testPlanner
 */
TEST(TestPathPlanner, testNodeHandle) {
//  Create a test node handle
ros::NodeHandle nh;
astar_plugin::AStarGlobalPlanner testPP(nh);
//  current node name
const char* node_name = "/testPlanner";
//  Returns true when robot name is testPlanner
EXPECT_EQ(node_name, ros::this_node::getName());
}

/**
 * @def TEST(TestPathPlanner, testConversionToMap)
 * @brief To check if the coordinates convert to Map
 */
TEST(TestPathPlanner, testConversionToMap) {
//  Initialise test object
astar_plugin::AStarGlobalPlanner testPP;
testPP.originX = 0;
testPP.originY = 0;
float point = 10.0;
float& indCoordinate = point;
//  Convert to map coordinates
testPP.convertToMapCoordinates(indCoordinate, indCoordinate);
std::vector<float> testCoordinates {10.0, 10.0};
//  Should return true when testcoordinates are equal to 10, 10
EXPECT_EQ(testCoordinates, testPP.getMapCoordinates(indCoordinate,
                                                    indCoordinate));
}

/**
 * @def TEST(TestPathPlanner, testCheckCellCoordinates)
 * @brief To check for the map coordinates from given coordinates
 */
TEST(TestPathPlanner, testCheckCellCoordinates) {
//  initialise test object
astar_plugin::AStarGlobalPlanner testPP;
testPP.originX = 0;
testPP.originY = 0;
float point = 10.0;
float& indCoordinate = point;
int index = 0;
testPP.width = 10;
testPP.resolution = 180.0;

//  fetch the cell coordinates using the index, and individual coordinates
testPP.getCellCoordinates(index, indCoordinate, indCoordinate);
std::vector<float> testCoordinates {0.0, 0.0};

//  Should return true when the testcoordinates are at origin
EXPECT_EQ(testCoordinates, testPP.getMapCoordinates(indCoordinate,
                                                    indCoordinate));
}



/**
 * @def TEST(TestPathPlanner, testCellValuesIndex)
 * @brief To check if the cell is free at a given index
 */
TEST(TestPathPlanner, testCellValuesIndex) {
astar_plugin::AStarGlobalPlanner testPP;
testPP.width = 10;
testPP.height = 10;
testPP.mapSize = testPP.width * testPP.height;
//  create a test occupancy grid
testPP.occupancyGridMap = new bool[testPP.mapSize];
//  Should return false for origin.
EXPECT_FALSE(testPP.isCellFree(0));

delete[] testPP.occupancyGridMap;
}

/**
 * @def TEST(TestPathPlanner, testCellValues)
 * @brief To check if the cells are free at given X,Y.
 */
TEST(TestPathPlanner, testCellValues) {
astar_plugin::AStarGlobalPlanner testPP;
testPP.width = 10;
testPP.height = 10;
testPP.mapSize = testPP.width * testPP.height;
//  Create a test occupancy grid
testPP.occupancyGridMap = new bool[testPP.mapSize];
//  Should return false for origin.
EXPECT_FALSE(testPP.isCellFree(0, 0));
delete[] testPP.occupancyGridMap;
}

/**
 * @def TEST(TestPathPlanner, testCellIndexCalculation)
 * @brief To check the cell index value
 */
TEST(TestPathPlanner, testCellIndexCalculation) {
astar_plugin::AStarGlobalPlanner testPP;
int res = 0;
//  Should return 0 for origin when no map params
EXPECT_EQ(res, testPP.calculateCellIndex(0, 0));
}

/**
 * @def TEST(TestPathPlanner, testCellIndex)
 * @brief To check for the given cell index
 */
TEST(TestPathPlanner, testCellIndex) {
astar_plugin::AStarGlobalPlanner testPP;
int res = 0;
//  index should be zero for origin.
EXPECT_EQ(res, testPP.getCellIndex(0, 0));
}

/**
 * @def TEST(TestPathPlanner, testRowIndex) 
 * @brief To check for the cell row index
 */
TEST(TestPathPlanner, testRowIndex) {
astar_plugin::AStarGlobalPlanner testPP;
testPP.width = 10;
int res = 0;
//  Should return 0 for the origin index.
EXPECT_EQ(res, testPP.getCellRowIndex(0));
}

/**
 * @def TEST(TestPathPlanner, testColIndex)
 * @brief To check for the cell column index
 */
TEST(TestPathPlanner, testColIndex) {
astar_plugin::AStarGlobalPlanner testPP;
testPP.width = 10;
int res = 0;
//  Should return 0 for the cell column index
EXPECT_EQ(res, testPP.getCellColIndex(0));
}

/**
 * @def TEST(TestPathPlanner, testFindPath) 
 * @brief To check if found path is correct at zero g_score
 */
TEST(TestPathPlanner, testFindPath) {
astar_plugin::AStarGlobalPlanner testPP;

testPP.value = 0;
// float infinity = std::numeric_limits<float>::infinity();
std::vector<float> g_test = {0};
std::vector<int> resF = {0};
//  Should return 0 for origin.
EXPECT_EQ(resF, testPP.findPath(0, 0, g_test));
}

// *
//  * @def TEST(TestPathPlanner, testFindPathNew)
//  * @brief To check if found path is correct at infinite g_score

TEST(TestPathPlanner, testFindPathNew) {
astar_plugin::AStarGlobalPlanner testPP;

testPP.value = 0;
float infinity = std::numeric_limits<float>::infinity();
std::vector<float> g_test = {infinity};
std::vector<int> resF = {0};
testPP.width = 10;
//  Should return 0 for origin.
EXPECT_EQ(resF, testPP.findPath(0, 0, g_test));
}

/**
 * @def TEST(TestPathPlanner, testFindPathNewFilled)
 * @brief To check if found path is correct
 */
TEST(TestPathPlanner, testFindPathNewFilled) {
astar_plugin::AStarGlobalPlanner testPP;

testPP.value = 0;
float infinity = std::numeric_limits<float>::infinity();
std::vector<float> g_test = {infinity};
std::vector<int> resF = {10};
testPP.width = 10;
//  Should return 10 for the given coordinate
EXPECT_EQ(resF, testPP.findPath(10, 10, g_test));
}

/**
 * @def TEST(TestPathPlanner, testFindPathNewExtreme)
 * @brief To check if the robot is able to twist on given topic
 */
TEST(TestPathPlanner, testFindPathNewExtreme) {
astar_plugin::AStarGlobalPlanner testPP;
testPP.value = 0;
float infinity = std::numeric_limits<float>::infinity();
std::vector<float> g_test = {infinity};
std::vector<int> resF = {1000};
testPP.width = 1;
//  Should return 1000 for extreme coordinates
EXPECT_EQ(resF, testPP.findPath(1000, 1000, g_test));
}

/**
 * @def TEST(TestPathPlanner, testRunAStar) 
 * @brief To check if A Star runs correctly.
 */
TEST(TestPathPlanner, testRunAStar) {
astar_plugin::AStarGlobalPlanner testPP;
testPP.value = 0;
float infinity = std::numeric_limits<float>::infinity();
std::vector<float> g_test = {infinity};
if (g_test[0] < 0)
  std::cout << "Test";
std::vector<int> resF = {0};
testPP.width = 10;
testPP.height = 10;
testPP.mapSize = testPP.width * testPP.height;
testPP.occupancyGridMap = new bool[testPP.mapSize];
//  Given the map, the result should be 0.
EXPECT_EQ(resF, testPP.runAStar(0, 0));
delete[] testPP.occupancyGridMap;
}

/**
 * @def TEST(TestPathPlanner, testStartGoalValidationFirst)
 * @brief To check if start and end goal are within map
 */
TEST(TestPathPlanner, testStartGoalValidationFirst) {
astar_plugin::AStarGlobalPlanner testPP;
testPP.width = 10;
testPP.height = 10;
testPP.mapSize = testPP.width * testPP.height;
//  Create a test occupancy grid
testPP.occupancyGridMap = new bool[testPP.mapSize];
//  Should return false for origin
EXPECT_FALSE(testPP.isStartAndGoalValid(0, 0));
delete[] testPP.occupancyGridMap;
}

/**
 * @def TEST(TestPathPlanner, testStartGoalValidationSecond) 
 * @brief To check if start and end goal are within map.
 */
TEST(TestPathPlanner, testStartGoalValidationSecond) {
astar_plugin::AStarGlobalPlanner testPP;
testPP.width = 10;
testPP.height = 10;
testPP.mapSize = testPP.width * testPP.height;
//  Create a test occupancy grid
testPP.occupancyGridMap = new bool[testPP.mapSize];
//  Should return false for the 1, 1
EXPECT_FALSE(testPP.isStartAndGoalValid(1, 1));
delete[] testPP.occupancyGridMap;
}

/**
 * @def TEST(TestPathPlanner, testAddNeighborsToOpenList)
 * @brief To check if open list is getting filled.
 */
TEST(TestPathPlanner, testAddNeighborsToOpenList) {
astar_plugin::AStarGlobalPlanner testPP;
int neighborCell = 1;
int goalCell = 3;
testPP.width = 5.0;
testPP.height = 5.0;
int mapSize = testPP.width*testPP.height;
float infinity = std::numeric_limits< float >::infinity();
std::vector<float> g_score;
//  fill the g_score vector with inf
g_score.assign(mapSize, infinity);
std::set<GridSquare> OPL;
//  Should return true if empty.
EXPECT_EQ(true, OPL.empty());
//  Should return the OPL list for the given neighbor cells.
testPP.addNeighborCellToOpenList(OPL, neighborCell, goalCell, g_score);
//  Should return false as the list is filled.
EXPECT_EQ(false, OPL.empty());
}
