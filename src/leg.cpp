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

#include <stdint.h>
#include <stdlib.h>
#include <vector>

#include <ros/ros.h>

#include "std_msgs/Float64.h"

#include "hexapod_controller/leg.h"

#include "hexapod_controller/calc_utils.h"

Leg::Leg(ros::NodeHandle &nhp, 
         std::string name, 
         Eigen::Matrix4f base_T,
         Eigen::ArrayXXf dh_params,
         Eigen::Vector3f joint_directions,
         Eigen::Vector3f theta_default)
    : leg_name(name), 
      base_T(base_T),
      dh_params(dh_params),
      joint_directions(joint_directions), 
      theta_default(theta_default) 
  {
  //spawn publishers for joint angles
  std::string names[5] = {"coxa", "femur", "tibia", "trochanter", "tarsus"};
  for(int i = 0; i < 3; ++i){
    joint_state_pub.push_back(nhp.advertise<std_msgs::Float64>("j_" + names[i] + "_" + leg_name, 12));
  }

  // set default pose
  forward_kinematics(Eigen::Matrix4f::Identity(), theta_default, 4,
                     foottip_default, nullptr);
}

/**
 * Function to advertise the theta_desired joint angles to the respective joint topics
 *
 */
//set_angles//{
void Leg::set_angles(void){
  std_msgs::Float64 msg;
 
  for(int i = 0; i < 3; ++i){
    msg.data = theta_desired[i] * joint_directions[i];
    joint_state_pub[i].publish(msg);
  }
}/*//}*/

/**
 * method to calculate the leg forward kinematics task (FKT) w.r.t. global reference frame
 *
 * @param[in] body_pose - 4x4 transformation matrix of the robot base w.r.t. global reference frame
 * @param[in] joint_angles - vector of three (coxa, femur, tibia) joint angles in radians
 * @param[in] dof_limit - number of DoFs to calculate the kinematics for
 * @param[out] world_pos - resulting position of the leg endpoint w.r.t. global reference frame
 * @param[out] Mi - reference to output vector of intermediate transformations (for visualization purposes)
 *
 * @return - true if the FKT was calculated correctly, false otherwise
 */
//forward kinematics//{
bool Leg::forward_kinematics(
                             const Eigen::Matrix4f &body_pose,
                             const Eigen::Vector3f &joint_angles,
                             const int dof_limit, Eigen::Vector3f &world_pos,
                             std::vector<Eigen::Matrix4f> *Mi) {

  // check validity of dof_limits
  if (dof_limit < 0 || dof_limit > dh_params.rows() + 1) {
    std::cerr << "Error FKT calc for " << dof_limit << " DOFs! "
              << dh_params.rows() + 1 << std::endl;
    return false;
  }

  // assign base leg transformation
  Eigen::Matrix4f Ai = body_pose * base_T;

  // save the intermediate result
  if (Mi) {
    Mi->push_back(Ai);
  }

  // calculate the kinematic chain
  for (int i = 1; i < dof_limit; ++i) {
    Eigen::Matrix4f Dh = dh_matrix(dh_params(i - 1, 0),
                                   dh_params(i - 1, 1) + joint_angles[i - 1],
                                   dh_params(i - 1, 2), dh_params(i - 1, 3));
    Ai = Ai * Dh;
    if (Mi) {
      Mi->push_back(Dh);
    }
  }
  // assign the resulting world position
  world_pos = Eigen::Vector3f(Ai(0, 3), Ai(1, 3), Ai(2, 3));

  return true;
} /*//}*/

/**
 * method to calculate the 3 DoF leg inverse kinematics task (IKT) w.r.t. global reference frame
 *
 * @param[in] body_pose - 4x4 transformation matrix of the robot base w.r.t. global reference frame
 * @param[in] world_pos - position of the leg endpoint w.r.t. global reference frame
 * @param[out] joint_angles - resulting vector of three (coxa, femur, tibia) joint angles in radians
 *
 * @return - true if the IKT was calculated correctly, false otherwise
 */
// inverse kinematics//{
bool Leg::inverse_kinematics(const Eigen::Matrix4f &body_pose,
                             const Eigen::Vector3f &world_pos,
                             Eigen::Vector3f &joint_angles) {

  joint_angles = Eigen::Vector3f::Zero();
  // get leg base
  Eigen::Matrix4f inv_T = (body_pose * base_T).inverse();
  // solution for coxa
  // get coordinates of the world_pos and world_orientation in coxa frame
  Eigen::Vector3f world_pos_cframe =
      (inv_T * (world_pos.homogeneous())).hnormalized();

  // extract coxa and trochanter angles
  float X = world_pos_cframe[0];
  float Y = world_pos_cframe[1];
  float Z = world_pos_cframe[2];
  joint_angles[0] = atan2(Y, X) - dh_params(0, 1);

  Eigen::Vector3f world_pos_fframe(sqrt(X * X + Y * Y) - dh_params(0, 3), Z, 0);

  float a2 = dh_params(1, 3);
  float a3 = dh_params(2, 3);
  float theta_2_off = dh_params(1, 1);
  float theta_3_off = dh_params(2, 1);

  // check for reachability of the position
  float f_len = world_pos_fframe.norm();
  if (f_len > (a2 + a3)) {
    return false;
  }

  // angles
  joint_angles[1] =
      acos((a2 * a2 - a3 * a3 + f_len * f_len) / (2 * a2 * f_len)) -
      atan(-world_pos_fframe[1] / world_pos_fframe[0]) - theta_2_off;
  joint_angles[2] = -M_PI +
                    acos((a2 * a2 + a3 * a3 - f_len * f_len) / (2 * a2 * a3)) -
                    theta_3_off;

  return true;
}
/*//}*/
