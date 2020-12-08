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
 *  @file    AstarPlanner.cpp
 *  @author  Arjun Srinivasan Ambalam
 *  @author  Rajeshwar N.S
 *  @date    12/07/2020
 *  @version 1.0
 *
 *  @brief Final Project - wanderer-bot (Search and collect objects)
 *
 *  @section DESCRIPTION
 *  
 *  A-star path planner has members that can be used directly into simulated world.
 */
#ifndef INCLUDE_ASTARPLANNER_HPP_
#define INCLUDE_ASTARPLANNER_HPP_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_core/base_global_planner.h>
#include <string>
#include <vector>
#include "OccupancyGrid.hpp"
#include "sensor_msgs/LaserScan.h"
#include "AstarPlanner.hpp"

PLUGINLIB_EXPORT_CLASS(astar_plugin::AStarPlanner, nav_core::BaseGlobalPlanner)

namespace astar_plugin {

  AStarPlanner::AStarPlanner() {
  }

  AStarPlanner::AStarPlanner(ros::NodeHandle &nh) {
    //  initialise node handle for this plugin
    ROSNodeHandle = nh;
  }

  AStarPlanner::AStarPlanner(std::string name,
                             costmap_2d::Costmap2DROS *cost_ros) {
    //  overridden function call for the node initialisation
    initialize(name, cost_ros);
  }

  void AStarPlanner::initialize(std::string name,
                                costmap_2d::Costmap2DROS *cost_ros) {
}

bool AStarPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                            const geometry_msgs::PoseStamped &goal,
                            std::vector<geometry_msgs::PoseStamped> &plan) {
  if (!initialized)
    return false;
  else
    return true;
}



std::vector<float> AStarPlanner::getMapCoordinates(float x, float y) {
  std::vector<float> coordinates {x, y};
  return coordinates;
}

int AStarPlanner::getCellIndex(float x, float y) {
  int cell = 0;
  //  returns cell index here
  return cell;
}




std::vector<int> AStarPlanner::runAStar(int startCell, int goalCell) {
  std::vector<int> bestPath = {0}
  return bestPath;
}

std::vector<int> AStarPlanner::findPath(int startCell,
                                        int goalCell,
                                        std::vector<float> g_score) {
  std::vector<int> bestPath = {0};
  return bestPath;
}


std::vector<int> AStarPlanner::constructPath(int startCell,
                                             int goalCell,
                                             std::vector<float> g_score) {
  std::vector<int> bestPath = {0};
  return bestPath;
}


std::vector<int> AStarPlanner::findFreeNeighborCell(int cell) {
  std::vector<int> freeNeighborCells = {0};

  return freeNeighborCells;
}


float AStarPlanner::getMoveToCellCost(int i1, int j1, int i2, int j2) {
  float moveCost = 1000000.0;

  return moveCost;
}



int AStarPlanner::calculateCellIndex(int i, int j) {
  return i;
}



int AStarPlanner::getCellColIndex(int index) {
  return index;
}
