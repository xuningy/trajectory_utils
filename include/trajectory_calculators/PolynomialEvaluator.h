/*
PolynomialEvaluator.
Copyright (C) 2021 Xuning Yang

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <limits>
#include <stdexcept>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <planning_representations/FlatState.h>

namespace traj_utils
{
  typedef Eigen::Matrix<double, 8, 1> Vec8_t;
  // get pose at time for a 7th order polynomial (8 coeffs)
  inline planner::FlatState getPoseAtTimePoly7(double t, const std::vector<Vec8_t, Eigen::aligned_allocator<Vec8_t>>& coeffs)
  {
    double tp2 = t*t;
    double tp3 = tp2*t;
    double tp4 = tp3*t;
    double tp5 = tp4*t;
    double tp6 = tp5*t;
    double tp7 = tp6*t;
    double tp8 = tp7*t;

    Eigen::Matrix<double, 5, 8> time_matrix;
    time_matrix <<  1, t, tp2, tp3, tp4, tp5, tp6, tp7,
              0, 1, 2*t, 3*tp2, 4*tp3, 5*tp4, 6*tp5, 7*tp6,
              0, 0, 2, 6*t, 12*tp2, 20*tp3, 30*tp4, 42*tp5,
              0, 0, 0, 6, 24*t, 60*tp2, 120*tp3, 210*tp4,
              0, 0, 0, 0, 24, 120*t, 360*tp2, 840*tp3;

    Eigen::Matrix<double, 5, 1> x_pose, y_pose, z_pose, yaw_pose;
    x_pose = time_matrix * coeffs[0];
    y_pose = time_matrix * coeffs[1];
    z_pose = time_matrix * coeffs[2];
    yaw_pose = time_matrix * coeffs[3];

    planner::FlatState pose;
    pose.t = t;

    pose.pos = Eigen::Vector3d(x_pose(0), y_pose(0), z_pose(0));
    pose.vel = Eigen::Vector3d(x_pose(1), y_pose(1), z_pose(1));
    pose.acc = Eigen::Vector3d(x_pose(2), y_pose(2), z_pose(2));
    pose.jerk = Eigen::Vector3d(x_pose(3), y_pose(3), z_pose(3));
    pose.snap = Eigen::Vector3d(x_pose(4), y_pose(4), z_pose(4));

    pose.yaw = yaw_pose(0);
    pose.dyaw = yaw_pose(1);
    pose.ddyaw = yaw_pose(2);
    pose.dddyaw = yaw_pose(3);
    // pose.rpy = Eigen::Vector3d(0, 0, yaw_pose(0));
    // pose.ang_vel = Eigen::Vector3d(0, 0, yaw_pose(1));

    return pose;
  }

  inline std::vector<planner::FlatState> getPosesAtTimePoly7(double T, const std::vector<Vec8_t, Eigen::aligned_allocator<Vec8_t>>& coeffs, double dt) {
    std::vector<planner::FlatState> states;
    for (double t = 0; t < T; t+=dt)
    {
      states.push_back(getPoseAtTimePoly7(t, coeffs));
    }
    return states;
  }

  inline std::vector<planner::FlatState> getPosesAtTimePoly7(double T, const std::vector<Vec8_t, Eigen::aligned_allocator<Vec8_t>>& coeffs, int N) {
    double dt = T/(N-1);
    return getPosesAtTimePoly7(T, coeffs, dt);
  }

  inline std::vector<Eigen::Vector3d> getPosAtTimePoly7(double T, const std::vector<Vec8_t, Eigen::aligned_allocator<Vec8_t>>& coeffs, double dt) {

    std::vector<Eigen::Vector3d> poses;
    for (double t = 0; t <= T+1e-7; t+=dt)
    {
      planner::FlatState state = getPoseAtTimePoly7(t, coeffs);
      poses.push_back(state.pos);
    }
    return poses;
  }

  inline std::vector<Eigen::Vector3d> getPosAtTimePoly7(double T, const std::vector<Vec8_t, Eigen::aligned_allocator<Vec8_t>>& coeffs, int N) {
    double dt = T/(N-1);
    return getPosAtTimePoly7(T, coeffs, dt);
  }
} // end namespace traj_utils
