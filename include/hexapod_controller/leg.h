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

#ifndef __LEG_H__
#define __LEG_H__

#include <ros/ros.h>

#include <stdlib.h>
#include <stdint.h>
#include <vector>

#include <Eigen/Dense>

class Leg{
  private:
    //name
    std::string leg_name; 

    //joint position publishers
    std::vector<ros::Publisher> joint_state_pub;

  public:

    //angle
    Eigen::Vector3f theta_real;
    Eigen::Vector3f theta_desired;
    Eigen::Vector3f theta_default;
    Eigen::Vector3f theta_new;

    //foottip
    Eigen::Vector3f foottip_real;
    Eigen::Vector3f foottip_desired;
    Eigen::Vector3f foottip_default;
    Eigen::Vector3f foottip_orig;
    Eigen::Vector3f foottip_new;
    Eigen::Vector3f foottip_delta;

    //DH matrix
    Eigen::ArrayXXf dh_params;
    
    //base transformation
    Eigen::Matrix4f base_T;

    //servo IDs
    Eigen::Vector3f joint_directions;
    
    //constructor
    Leg(ros::NodeHandle &nhp,
        std::string name, 
        Eigen::Matrix4f base_T, 
        Eigen::ArrayXXf dh_params,
        Eigen::Vector3f joint_directions, 
        Eigen::Vector3f theta_default);
    
    //set angles 
    void set_angles(void);

    //forward kinematics
    bool forward_kinematics(
        const Eigen::Matrix4f &body_pose, 
        const Eigen::Vector3f &joint_angles, 
        const int dof_limit, 
        Eigen::Vector3f &world_pos,
        std::vector<Eigen::Matrix4f> *Mi);

    //inverse kinematics
    bool inverse_kinematics(
        const Eigen::Matrix4f &body_pose,
        const Eigen::Vector3f &world_pos, 
        Eigen::Vector3f &joint_angles);

};
#endif
