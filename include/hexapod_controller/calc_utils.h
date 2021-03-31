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

#ifndef __CALC_UTILS__
#define __CALC_UTILS__

#include <Eigen/Dense>
#include <vector>

/**
 * Compute the DH matrix given the DH parameters
 *
 * @param[in] alpha, phi, d, a - the DH parameters of the link
 * @return 4x4 DH matrix
 */
static Eigen::Matrix4f dh_matrix(float alpha, float phi, float d, float a){
    Eigen::Matrix4f M;
    M << cos(phi), -sin(phi)*cos(alpha), sin(phi)*sin(alpha), a*cos(phi),
         sin(phi), cos(phi)*cos(alpha), -cos(phi)*sin(alpha), a*sin(phi),
         0, sin(alpha), cos(alpha), d,
         0, 0, 0, 1.0;
    return M;
}

/**
 * Returns the rotation matrix given Euler angles
 *
 *  @param[in] yaw, pitch, roll - Euler angles
 *  @return 3x3 rotation matrix
 */
static Eigen::Matrix3f rotation_matrix(float yaw, float pitch, float roll){
  Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
  Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());

  Eigen::Quaternion<float> q = yawAngle * pitchAngle * rollAngle;

  Eigen::Matrix3f rotationMatrix = q.matrix();
  return rotationMatrix;
}

/**
 * Compute least squares rigid transform fit of two sets of matching points
 *
 * @param[in] src - source coordinate vectors
 * @param[in] dst - destination coordinate vectors
 * @return 4x4 rigid transformation matrix
 */
static Eigen::Matrix4f rigid_transform_3D(std::vector<Eigen::Vector3f> &src, std::vector<Eigen::Vector3f> &dst){
  
	  assert(src.size() == dst.size());
    int pairSize = src.size();
    Eigen::Vector3f center_src(0, 0, 0), center_dst(0, 0, 0);
    for (int i=0; i<pairSize; ++i)
    {
      center_src += src[i];
      center_dst += dst[i];
    }
    center_src /= (double)pairSize;
    center_dst /= (double)pairSize;

    Eigen::MatrixXf S(pairSize, 3), D(pairSize, 3);
    for (int i=0; i<pairSize; ++i)
    {
      for (int j=0; j<3; ++j)
        S(i, j) = src[i][j] - center_src[j];
      for (int j=0; j<3; ++j)
        D(i, j) = dst[i][j] - center_dst[j];
    }
    Eigen::MatrixXf Dt = D.transpose();
    Eigen::Matrix3f H = Dt*S;
    Eigen::Matrix3f W, U, V;

    Eigen::JacobiSVD<Eigen::MatrixXf> svd;
    Eigen::MatrixXf H_(3, 3);
    for (int i=0; i<3; ++i) for (int j=0; j<3; ++j) H_(i, j) = H(i, j);
    svd.compute(H_, Eigen::ComputeThinU | Eigen::ComputeThinV );
    if (!svd.computeU() || !svd.computeV()) {
      std::cerr << "decomposition error" << std::endl;
      return Eigen::Matrix4f::Identity();
    }
    Eigen::Matrix3f Vt = svd.matrixV().transpose();
    Eigen::Matrix3f R = svd.matrixU()*Vt;
    if(R.determinant() < 0){
      //std::cerr << "reflection detected" << std::endl;
      Vt.block<3,1>(0,2) *= -1;
      R = svd.matrixU()*Vt;
    }

    Eigen::Vector3f t = center_dst - R*center_src;	
    
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity(4,4); 
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = t;
    return T;
}

#endif
