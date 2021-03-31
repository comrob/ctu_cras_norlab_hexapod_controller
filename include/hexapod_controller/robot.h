/*
 * Copyright (c) 2021, Computational Robotics Laboratory, AI center, FEE, 
 * CTU in Prague, Czech Republic
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <stdlib.h>
#include <stdint.h>
#include <vector>
#include <string>

#include <Eigen/Dense>

#include <ros/ros.h>

#include <jsoncpp/json/json.h>

#include "leg.h"

class Robot{
  public:
    //legs
    int n_legs;
    std::vector<Leg> legs;

    //body pose
    Eigen::Matrix4f body_pose;
    
    //body anchors
    std::vector<Eigen::Vector3f> body_anchors;

    //default height
    float default_height;

    //default leg reach
    float default_step_height;

    //default leg reach
    float leg_reach;

    //constructor
    Robot(ros::NodeHandle &nhp, const std::string robot_description_filename);

  private:
    //parse vector from json file
    Eigen::VectorXf parse_vector(Json::Value root);
    //parse array from json file
    Eigen::ArrayXXf parse_array(Json::Value root);
};


#endif
