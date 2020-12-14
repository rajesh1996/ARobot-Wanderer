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
 *  @file    Collector.cpp
 *  @author  Arjun Srinivasan Ambalam
 *  @author  Rajeshwar N.S
 *  @date    12/07/2020
 *  @version 1.0
 *
 *  @brief Final Project - wanderer-bot (Finding objects)
 *
 *  @section DESCRIPTION
 *  
 *  Class has members that can be used directly into simulated world.
 */
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "../include/SpawnCollect.hpp"
#include "../include/Randomizer.hpp"
#include "../include/Collector.hpp"


// Collector::Collector() {
// }
// Collector::~Collector() {
// }

// bool Collector::collector() {
//   bool flag = true;


// return flag;
// }

// int main(int argc, char** argv) {
//   return 0;
// }



typedef actionlib::SimpleActionClient
<move_base_msgs::MoveBaseAction> MoveBaseClient;

Collector::Collector() {
}
Collector::~Collector() {
}

bool Collector::collector() {
  bool flag = true;
  // double xo = 0;
  // double yo = 0;
  // Randomizer r;
  // SpawnCollect s;
  // double xr = r.randomizeX();
  // double yr = r.randomizeY();
  // s.spawn(xr, yr, 1);

  // double xn = -7.0 + xr;
  // double yn = -6.5 + yr;
  // xn = r.xOffset(xo, xr, xn);
  // yn = r.yOffset(yo, yr, yn);

  // tell the action client  to spin a thread
  MoveBaseClient ac("move_base", true);

  // wait for action server
  while (!ac.waitForServer(ros::Duration(10.0))) {
    ROS_INFO_STREAM("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // sending goal
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 1;
  goal.target_pose.pose.position.y = 1;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO_STREAM("Sending goal");
  ac.sendGoal(goal);

  // ac.waitForResult(ros::Duration(17 + 2.3 * abs(6.5 + yr)));
  // //  if the robot reaches successfully then it will remove the trash
  // if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
  //   ROS_INFO_STREAM("Hooray, Trash is collected!");
  //   s.collect(1);
  // } else {
  //   ROS_INFO_STREAM("The base is in vicinity to the Object. Hence collected!");
  //   s.collect(1);
  // }
return flag;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_navigation_goals");
  Collector book;
  book.collector();
  return 0;
}
