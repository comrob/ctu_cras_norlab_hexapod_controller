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

#include "hexapod_controller/robot.h"

#include "hexapod_controller/calc_utils.h"

#include <fstream>

Robot::Robot(ros::NodeHandle &nhp, const std::string robot_description_filename){

  //prepare the body pose
  body_pose = Eigen::Matrix4f::Identity();

  //json parser - parse the robot structure and create the legs//{
  Json::Value robot_description;
  std::ifstream robot_description_file(robot_description_filename, std::ifstream::binary);
  if (robot_description_file.fail()){
     throw std::runtime_error("..");
  }

  //read the robot description
  robot_description_file >> robot_description;

  //parse individual legs
  n_legs = robot_description["legs"].size();
  for(int l = 0; l < n_legs; ++l){

    Json::Value leg_description = robot_description["legs"][l];
    //parse name
    std::string leg_name = leg_description["name"].asString();

    //parse base transformation
    Eigen::Matrix4f base_T = parse_array(leg_description["base_T"]);

    //parse Denavit-Hartenberg parameters
    Eigen::ArrayXXf dh_params = parse_array(leg_description["dh_params"]);
    
    //parse theta default
    Eigen::Vector3f theta_default = parse_vector(leg_description["theta_default"]);

    //parse joint dirs
    Eigen::Vector3f joint_dirs = parse_vector(leg_description["joint_dirs"]);

    //create leg
    legs.push_back(Leg(nhp, leg_name, base_T, dh_params, joint_dirs, theta_default));
    //create body anchors
    body_anchors.push_back(base_T.block<3,1>(0,3));
  }/*//}*/

  //analyze the leg workspace and assign appropriate value of the default height,
  //maximum leg reach, and step height
  std::vector<Eigen::Vector3f> footholds;
  default_step_height = 0;
  for(int i = 0; i < n_legs; ++i){
    legs[i].forward_kinematics(body_pose, legs[i].theta_default, 4, legs[i].foottip_default, nullptr);
    footholds.push_back(legs[i].foottip_default);

    Eigen::Vector3f delta_foottip(0,0,0);
    Eigen::Vector3f joint_angles;
    while(legs[i].inverse_kinematics(body_pose, 
          legs[i].foottip_default + delta_foottip,
          joint_angles)){
        //increment in z-direction to assess the highest point above the default foothold the leg can reach
        delta_foottip = delta_foottip + Eigen::Vector3f(0,0,1);
    }
    default_step_height = (default_step_height < delta_foottip.norm()) ? delta_foottip.norm() : default_step_height;
  }

  //calculate the default height of the robot
  Eigen::Matrix4f A = rigid_transform_3D(body_anchors,footholds);
  default_height = A.block<3,1>(0,3).norm();
  //calculate the max reach of the legs
  leg_reach = (footholds[0] - footholds[1]).norm();
  for(int i = 1; i < n_legs; ++i){
    for(int j = 1; j < n_legs; ++j){
      float dist = (footholds[i] - footholds[j]).norm();
      if (i != j && leg_reach > dist){
        leg_reach = dist;
      }
    }
  }
  //assign half of the shortest distance as the maximum leg reach
  leg_reach = 0.5*leg_reach;

  //assign half of the maax step height as default
  default_step_height = 0.5*default_step_height;
}

//parse vector from json file//{
Eigen::VectorXf Robot::parse_vector(Json::Value root){
  //get the dimensions
  int l = root.size();

  Eigen::VectorXf vec(l);

  for(int i = 0; i < l; ++i){
    vec[i] = root[i].asFloat();
  }
  return vec;
}/*//}*/

//parse array from json file//{
Eigen::ArrayXXf Robot::parse_array(Json::Value root){
  //get the dimensions
  int h = root.size();
  int w = root[0].size();

  Eigen::ArrayXXf arr = Eigen::ArrayXXf(h, w);

  //parse the individual values
  for(int i = 0; i < h; ++i){
    for(int j = 0; j < w; ++j){
      arr(i,j) = root[i][j].asFloat(); 
    }
  }
  return arr;
}/*//}*/


