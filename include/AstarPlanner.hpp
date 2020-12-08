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
 *  @file    Collector.hpp
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

namespace astar_plugin {
class AStarPlanner : public nav_core::BaseGlobalPlanner {
 public:
    int value;
    int mapSize;  //  size of the occupancy grid map
    ros::NodeHandle ROSNodeHandle;  //  Nodehandle object
    float originX;
    float originY;
    float resolution;
    costmap_2d::Costmap2DROS *costmap_ros;
    costmap_2d::Costmap2D *costmap;
    bool initialized;  //  variable to check if path planner is initialised
    int width;
    int height;

    /**
     *   @brief Constructor of class AStar planner
     *   @param none
     *   @return none
     */
    AStarPlanner();

    /**
     *   @brief Overloaded constructor to call ros node handle.
     *   @param ros::NodeHandle
     *   @return none
     */
    explicit AStarPlanner(ros::NodeHandle &);

    /**
     *   @brief Overloaded constructor to initialise 2D cost map 
     *   @param string, name
     *   @param costmap_2d::Costmap2DROS, ROS 2D cost map
     *   @return none
     */
    AStarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

    /**
     *   @brief Function inherited from base class to initialise map
     *   @param string, name
     *   @param costmap_2d::Costmap2DROS, ROS 2D cost map
     *   @return none
     */
    void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

    /**
     *   @brief Function to make a plan to reach the goal 
     *   @param const geometry_msgs::PoseStamped, start pose
     *   @param const geometry_msgs::PoseStamped, goal pose
     *   @param std::vector<geometry_msgs::PoseStamped>, vector plan to reach
     *   @return bool, returns true if plan exists
     */
    bool makePlan(const geometry_msgs::PoseStamped &start,
                  const geometry_msgs::PoseStamped &goal,
                  std::vector<geometry_msgs::PoseStamped> &plan);
    /**
     *   @brief Function to run astar algorithm
     *   @param int, start cell
     *   @param int, goal cell
     *   @return std::vector<int>, returns best path coordinates
     */
    std::vector<int> runAStar(int startCell, int goalCell);

    /**
     *   @brief Function to find path to reach the goal 
     *   @param int, start cell
     *   @param int, goal cell
     *   @param std::vector<float>, g function value for each cell
     *   @return std::vector<int>, if correct, then best path else empty path
     */
    std::vector<int> findPath(int startCell, int goalCell,
                              std::vector<float> g_score);

    /**
     *   @brief Function to construct a path to reach the goal 
     *   @param int, start cell
     *   @param int, goal cell
     *   @param std::vector<float>, g function value for each cell
     *   @return std::vector<int>, if correct, then best path else empty path
     */
    std::vector<int> constructPath(int startCell, int goalCell,
                                   std::vector<float> g_score);

    /**
     *   @brief Function to get map coordinates
     *   @param float, x coordinate
     *   @param float, y coordinate
     *   @return std::vector<float> returns map coordinates
     */
    std::vector<float> getMapCoordinates(float x, float y);



    /**
     *   @brief Function to calculate cell index
     *   @param int, cell y value 
     *   @param int, cell x value
     *   @return int, cell index
     */
    int calculateCellIndex(int i, int j);




    /**
     *   @brief Function to check if the cell is free
     *   @param int, cell index value 
     *   @return bool, returns true if free
     */
    bool isCellFree(int cellIndex);


   /**
     *   @brief Overloaded function to get moving cost to a cell
     *   @param int, first cell x index
     *   @param int, first cell y index
     *   @param int, second cell x index
     *   @param int, second cell y index
     *   @return float, cell cost
     */
    float getMoveToCellCost(int i1, int j1, int i2, int j2);

    /**
     *   @brief Function to find free neighbouring cell to traverse
     *   @param int, previous cell index
     *   @return std::vector<int>, new cell indices
     */
    std::vector<int> findFreeNeighborCell(int cellIndex);



    /**
     *   @brief Function to get cell index
     *   @param int, x coordinate
     *   @param int, y coordinate
     *   @return int, index of cell
     */
    int getCellIndex(float x, float y);
};
};  //  namespace astar_plugin

#endif  //  INCLUDE_ASTARPLANNER_HPP_
