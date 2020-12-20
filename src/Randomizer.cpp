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
 *  @file    Randomizer.cpp
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

#include <cstdlib>
#include <ctime>
#include <vector>
#include <random>
#include "../include/Randomizer.hpp"

Randomizer::Randomizer() {
            xc = 0;
            yc = 0;
}

Randomizer::~Randomizer() {
}

std::vector<double> Randomizer::randomizecoord() {
double xc;
double yc;
double zc;
std::vector<double> cord;
std::vector<int> v = {1, 2, 3};
std::random_device random_device;
std::mt19937 engine{random_device()};
std::uniform_int_distribution<int> dist(0, v.size() - 1);
int random_element = v[dist(engine)];
if (random_element == 1) {
xc = -7.245;
yc = 4.9677;
zc = 0.4458;
}
if (random_element ==2) {
xc = 7.2625;
yc = 5.090;
zc = 0.0699;
}

if (random_element == 3) {
xc = 8.367830;
yc = 0.2700;
zc = 0.9203;
}
cord = {xc, yc, zc};
return cord;
}



double Randomizer::xOffset(double xn) {
// add offset to goal location
return xn;
}

double Randomizer::yOffset(double yn) {
// add offset to goal location
return yn-0.6;
}
