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
 */

#ifndef INCLUDE_RANDOMIZER_HPP_
#define INCLUDE_RANDOMIZER_HPP_

#include <vector>
class Randomizer {
 public:
  double xc, yc;

  /**
   *   @brief Constructor of class Randomizer
   *   @param none
   *   @return none
   */
  Randomizer();

  /**
   *   @brief Destructor of class Randomizer
   *   @param none
   *   @return none
   */
  ~Randomizer();

  /**
   *   @brief function to randomize x
   *   @param none
   *   @return double xr
   */
  std::vector<double> randomizecoord();



  /**
   *   @brief function to offset x
   *   @param double xo, robot origin
   *   @param double xr, trash
   *   @param double xn, target
   *   @return double xn, new traget
   */
  double xOffset(double xn);

  /**
   *   @brief function to offset y
   *   @param double yo, robot origin
   *   @param double yr, trash
   *   @param double yn, traget
   *   @return double yn, new target
   */
  double yOffset(double yn);
};

#endif  //  INCLUDE_RANDOMIZER_HPP_
