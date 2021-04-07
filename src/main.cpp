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

#include <ros/ros.h>

#include <stdlib.h>
#include <vector>
#include <string>

#include "hexapod_controller/calc_utils.h"
#include "hexapod_controller/leg.h"
#include "hexapod_controller/robot.h"

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>

#define CONTROL_RATE 25

typedef struct{
  int numGaitPhases;
  std::vector<std::vector<int>> phases;
} Gait;

//velocity commands
float v_x = 0, v_y = 0, omega_z = 0;
//normalized gravity vector for steep inclines compensation
Eigen::Vector3f g_sampled(0,0,-1);

float step_time = 2.0; //duration of a single step [seconds]
float step_clearance = 0.0; //step clearance [m]
float body_clearance = 0.0; //body clearance [m]
float t_con = 1/(float)CONTROL_RATE; //control period [Hz]

//broadcast tf msg//{
void broadcast_tf_msg(tf2_ros::TransformBroadcaster br, std::string frame, std::string subframe, Eigen::Matrix4f T){
  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = ros::Time::now();
  tf_msg.header.frame_id = frame;
  tf_msg.child_frame_id = subframe;
  tf_msg.transform.translation.x = T(0,3);
  tf_msg.transform.translation.y = T(1,3);
  tf_msg.transform.translation.z = T(2,3);

  Eigen::Quaternionf q(T.block<3,3>(0,0));
  q.normalize();
  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();
  tf_msg.transform.rotation.w = q.w();

  br.sendTransform(tf_msg);
}/*//}*/

//velocity command callback//{
void steering_callback(const geometry_msgs::Twist::ConstPtr& msg){
  v_x = std::min(std::max((float)msg->linear.x,-1.0f),1.0f);
  v_y = std::min(std::max((float)msg->linear.y,-1.0f),1.0f);
  omega_z = msg->angular.z;
  ROS_INFO_THROTTLE(1.0, "[hexapod_controller] velocity command: %.2f, %.2f, %.2f",v_x,v_y,omega_z);
}/*//}*/

//step height command callback//{
void step_clearance_callback(const std_msgs::Float64::ConstPtr& msg){
  step_clearance = (msg->data > 0) ? msg->data : step_clearance;
  ROS_INFO("[hexapod_controller] step clearance set to %.2f", step_clearance);
}/*//}*/

//body clearance command callback//{
void body_clearance_callback(const std_msgs::Float64::ConstPtr& msg){
  body_clearance = (msg->data > 0) ? msg->data : body_clearance;
  ROS_INFO("[hexapod_controller] body clearance set to %.2f", body_clearance);
}/*//}*/

int main(int argc, char *argv[]){

  //ROS init
	ros::init(argc, argv, "hexapod_controller");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  //initialize the velocity command subscriber
  ros::Subscriber command_sub;
	command_sub = nh.subscribe("cmd_vel", 1, steering_callback);

  //initialize the step_clearance and body_clearance command subscribers
  ros::Subscriber step_clearance_sub;
	step_clearance_sub = nh.subscribe("step_clearance", 1, step_clearance_callback);
  ros::Subscriber body_clearance_sub;
	body_clearance_sub = nh.subscribe("body_clearance", 1, body_clearance_callback);

  //transform broadcaster
  static tf2_ros::TransformBroadcaster br;

  //get the robot description
  std::string robot_description_filename;
  nhp.getParam("robot_description", robot_description_filename);

  //create the robot
  Robot robot(nhp, robot_description_filename);

  //assign the default step height
  step_clearance = robot.default_step_height;
  body_clearance = robot.default_height;

  //READ the real angles (in case of simulation assign the zero angles)
  for(int i = 0; i < robot.n_legs; ++i){
    Leg *leg = &robot.legs[i];
    leg->theta_real = Eigen::Vector3f(0,0,0);
  }

  //Setup GAIT//{
  Gait gait;
  gait.phases.push_back(
      std::vector<int>({0,3,4}));
  gait.phases.push_back(
      std::vector<int>({1,2,5}));/*//}*/

  gait.numGaitPhases = gait.phases.size();

  while(ros::ok()){
    //determine swinging legs
    for (int gaitPhase = 0; gaitPhase < gait.numGaitPhases; gaitPhase++) {

      //reset the body pose
      robot.body_pose = Eigen::Matrix4f::Identity();
      for (int i = 0; i < robot.n_legs; ++i) {
        Leg *leg = &robot.legs[i];
        leg->forward_kinematics(robot.body_pose, leg->theta_real, 4, leg->foottip_real, nullptr);
      }

      int n_path = (int)((float)step_time/(float)gait.numGaitPhases/(float)t_con);
      int k_step = 0;
      //assign the swinging legs
      std::vector<int> swinging_legs = gait.phases[gaitPhase];

      //calculate pose to pose transformation based on the velocity command
      Eigen::Matrix4f pose_to_pose_T = Eigen::Matrix4f::Identity();
      pose_to_pose_T.block<3, 3>(0, 0) =
          rotation_matrix(omega_z , 0, 0);
      pose_to_pose_T(0, 3) = v_x * robot.leg_reach;
      pose_to_pose_T(1, 3) = v_y * robot.leg_reach;

      pose_to_pose_T = robot.body_pose*pose_to_pose_T;
      //std::cout << pose_to_pose_T << std::endl;

      //calculate the new footholds//{
      for(int i = 0; i < robot.n_legs; ++i){
        Leg *leg = &robot.legs[i];

        //save the current leg foottip positions at the beginning of gait phase
        leg->foottip_orig = leg->foottip_real;

        if ( std::find(swinging_legs.begin(), swinging_legs.end(), i) != swinging_legs.end()){
          //if the leg is supposed to swing, calculate its new position
          leg->foottip_new = (pose_to_pose_T * leg->foottip_default.homogeneous()).hnormalized();
          leg->foottip_delta = (-leg->foottip_orig + leg->foottip_new)/(float)n_path;
        } else {
          //stay at place
          leg->foottip_new = leg->foottip_orig;
          leg->foottip_delta = Eigen::Vector3f(0,0,0);
        }
      }/*//}*/

      //main loop
      ros::Rate rate(CONTROL_RATE);
      bool finished = false;
      while (!finished && ros::ok()){
        ros::spinOnce();

        //leg trajectory interpolation//{
        std::vector<Eigen::Vector3f> footholds;

        //calculate the gait phase
        float denom = (float) k_step/(float) n_path;
        float step_phase = (denom < 0.5) ? step_clearance*2*denom : step_clearance*2*(1-denom);

        for (int i = 0; i < robot.n_legs; ++i) {
          Leg *leg = &robot.legs[i];

          // select foottip position
          Eigen::Vector3f current_foottip;
          
          if ( std::find(swinging_legs.begin(), swinging_legs.end(), i) != swinging_legs.end()){
            //if the leg is supposed to swing, calculate its new position
            current_foottip = leg->foottip_orig + leg->foottip_delta*k_step;
            current_foottip[2] += step_phase;
          } else {
            //if not, hold the position
            current_foottip = leg->foottip_orig;
          }
          leg->foottip_desired = current_foottip;
          //body_pose
          footholds.push_back(current_foottip);
        }/*//}*/

        // body pose adjustment based on the current footholds
        robot.body_pose = rigid_transform_3D(robot.body_anchors,footholds);

        // project in the opposite direction of gravity
        robot.body_pose(2,3) -= step_phase/2.0;
        robot.body_pose.block<3, 1>(0, 3) -= g_sampled*body_clearance;

        //Perform the motion
        for (int i = 0; i < robot.n_legs; ++i) {
          Leg *leg = &robot.legs[i];

          // calculate the IKT
          Eigen::Vector3f j_angles;
          if(!leg->inverse_kinematics(robot.body_pose, leg->foottip_desired, j_angles)){
            ROS_ERROR("[hexapod_controller] IKT error");
            j_angles = leg->theta_real;
          }
          leg->theta_desired = j_angles;
        }
        
        //SET servo positions
        for (int i = 0; i < robot.n_legs; ++i) {
          Leg *leg = &robot.legs[i];
          leg->set_angles();
        }

        //READ servo positions and react to terrain irregularities
        for (int i = 0; i < robot.n_legs; ++i) {
          Leg *leg = &robot.legs[i];
          //READING in simulation ommited
          leg->theta_real = leg->theta_desired;
        }
        
        //finish the gait cycle
        if(k_step++ > n_path || swinging_legs.empty()){finished=true;}
        rate.sleep();
      }
    }
  }

  return 0;
}
