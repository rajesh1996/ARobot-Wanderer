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
 *  @file    Navigatebot.hpp
 *  @author  Arjun Srinivasan Ambalam
 *  @author  Rajeshwar N.S
 *  @date    12/07/2020
 *  @version 1.0
 *
 *  @brief Final Project - wanderer-bot (Search and collect objects)
 *
 *  @section DESCRIPTION
 *  
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

#include "../include/Navigatebot.hpp"

Navigatebot::Navigatebot() {
}

Navigatebot::~Navigatebot() {
}

void Navigatebot::twistRobot(const geometry_msgs::TwistConstPtr &msg) {
  //  Set geometry messages to the robot
    //  Set geometry messages to the robot
  double transVelocity = msg->linear.x;
  double rotVelocity = msg->angular.z;
  double velDiff = (0.143 * rotVelocity) / 2.0;
  double leftPower = (transVelocity + velDiff) / 0.076;
  double rightPower = (transVelocity - velDiff) / 0.076;
  //  check for individual node
  ROS_INFO_STREAM("\n Left wheel: " << leftPower
                  << ",  Right wheel: "<< rightPower << "\n");
}

int Navigatebot::start(bool flag) {
  //  initialise node handle
  ros::NodeHandle nh;
  if (flag) {
    //  start a subcriber to the given topic
    ros::Subscriber sub =
        nh.subscribe("/cmd_vel", 1000,
        &Navigatebot::twistRobot, this);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
    }
  } else {
    //  for testing of the node, this is fake subscriber that publishes for 5
    ros::Subscriber sub =
        nh.subscribe("/cmd_vel", 5,
        &Navigatebot::twistRobot, this);
  }

  return 0;
}
