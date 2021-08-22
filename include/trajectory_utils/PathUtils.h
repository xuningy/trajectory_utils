/*
PathUtils.
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
#include <Eigen/Eigen>

#include <planning_representations/FlatState.h>

namespace path_utils
{
  // compute length of the whole path.
  inline double Length(const std::vector<Eigen::Vector3d>& path)
  {
    double length = 0.0;
    if (path.size() < 2) return length;

    for (size_t i = 0; i < path.size() - 1; ++i) {
      length += (path[i + 1] - path[i]).norm();
    }
    return length;
  }

  // compute length of the whole path.
  inline double Length(const std::vector<planner::FlatState>& path)
  {
    double length = 0.0;
    if (path.size() < 2) return length;

    for (size_t i = 0; i < path.size() - 1; ++i) {
      length += (path[i + 1].pos - path[i].pos).norm();
    }
    return length;
  }

  // compute segment distances
  inline std::vector<double> SegmentLength(const std::vector<Eigen::Vector3d>& path)
  {
    if (path.size() == 0)
      throw std::invalid_argument("[path_utils::SegmentLength] path is an empty vector!");
    if (path.size() == 1)
      throw std::invalid_argument("[path_utils::SegmentLength] path contains 1 element, cannot compute segment lengths!");

    std::vector<double> dist;
    for (size_t i = 1; i < path.size() - 1; ++i) {
      dist.push_back((path[i] - path[i-1]).norm());
    }
    return dist;
  }

  // rediscretization of a path given a fixed number of points along the line
  inline std::vector<Eigen::Vector3d> discretizePath(const std::vector<Eigen::Vector3d>& path, int pt_num)
  {
    // if the path contains points that are identical (first and last points are the same)
    if (path.front() == path.back())
    {
      std::cout << "discretize path: same point!" << std::endl;
      std::vector<Eigen::Vector3d> dis_path;
      for (int i = 0; i < pt_num; i++)
      {
        dis_path.push_back(path.front());
      }

      return dis_path;

    }

    // else, discretize as normal
    std::vector<double> len_list;
    len_list.push_back(0.0);

    for (size_t i = 0; i < path.size() - 1; ++i) {
      double inc_l = (path[i + 1] - path[i]).norm();
      len_list.push_back(inc_l + len_list[i]);
    }

    // calc pt_num points along the path
    double len_total = len_list.back();
    double dl = len_total / double(pt_num - 1);
    double cur_l;

    std::vector<Eigen::Vector3d> dis_path;
    for (int i = 0; i < pt_num; ++i) {
      cur_l = double(i) * dl;

      // find the range cur_l in
      int idx = -1;
      for (size_t j = 0; j < len_list.size() - 1; ++j) {
        if (cur_l >= len_list[j] - 1e-4 && cur_l <= len_list[j + 1] + 1e-4) {
          idx = j;
          break;
        }
      }

      // find lambda and interpolate
      double lambda = (cur_l - len_list[idx]) / (len_list[idx + 1] - len_list[idx]);
      Eigen::Vector3d inter_pt = (1 - lambda) * path[idx] + lambda * path[idx + 1];
      dis_path.push_back(inter_pt);
    }

    return dis_path;
  }

  // rediscretization of a path given a fixed number of points along the line
  // version where the index vector is returned as well to show where the old path points are located. (if the point is between two idx, it should be the first one that is added). The length of the idx_vec is the same as path.
  inline std::vector<Eigen::Vector3d> discretizePath(const std::vector<Eigen::Vector3d>& path, int pt_num, std::vector<int>& idx_vec)
  {
    // if the path contains points that are identical (first and last points are the same)
    if (path.front() == path.back())
    {
      std::cout << "discretize path: same point!" << std::endl;
      std::vector<Eigen::Vector3d> dis_path;
      for (int i = 0; i < pt_num; i++)
      {
        dis_path.push_back(path.front());
        idx_vec.push_back(0); // only the first point is used.
      }

      return dis_path;

    }

    // else, discretize as normal
    std::vector<double> len_list;
    len_list.push_back(0.0);

    // tracking indices
    idx_vec.push_back(0);
    int prev_idx = 0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
      double inc_l = (path[i + 1] - path[i]).norm();
      len_list.push_back(inc_l + len_list[i]);
    }

    // calc pt_num points along the path
    double len_total = len_list.back();
    double dl = len_total / double(pt_num - 1);
    double cur_l;

    std::vector<Eigen::Vector3d> dis_path;
    for (int i = 0; i < pt_num; ++i) {
      cur_l = double(i) * dl;

      // find the range cur_l in
      int idx = -1;
      for (size_t j = 0; j < len_list.size() - 1; ++j) {
        if (cur_l >= len_list[j] - 1e-4 && cur_l <= len_list[j + 1] + 1e-4) {
          idx = j;
          break;
        }
      }

      // to extract indices where the values are.
      if (idx > prev_idx) {
        // push back prev_idx (k = idx - prev_idx) times
        int K = idx-prev_idx;
        for (int k = 0; k < K; k++) {
          idx_vec.push_back(i);
        }
        prev_idx = idx;
      }

      // find lambda and interpolate
      double lambda = (cur_l - len_list[idx]) / (len_list[idx + 1] - len_list[idx]);
      Eigen::Vector3d inter_pt = (1 - lambda) * path[idx] + lambda * path[idx + 1];
      dis_path.push_back(inter_pt);

    }
    // last point in the interpolation isn't added
    idx_vec.push_back(dis_path.size()-1);

    // check that lengths are the ssame
    if (idx_vec.size() != path.size()) {
      std::cout << "idx_vec length: " << idx_vec.size() << " path length: " << path.size() << " they should be the same! " << std::endl;
    }

    return dis_path;
  }


  // rediscretize a path
  inline std::vector<planner::FlatState> discretizePath(const std::vector<planner::FlatState>& path, int pt_num)
  {
    // convert it into a path first
    std::vector<Eigen::Vector3d> path_temp;
    for (auto& point : path)
    {
      path_temp.push_back(point.pos);
    }

    auto rediscretized_path_temp = discretizePath(path_temp, pt_num);

    // convert it back into state_t; however the higher derivative info will be lost.

    std::vector<planner::FlatState> rediscretized_path;
    for (auto& point : rediscretized_path_temp)
    {
      planner::FlatState state;
      state.pos = Eigen::Vector3d(point(0), point(1), point(2));
      rediscretized_path.push_back(state);
    }

    return rediscretized_path;
  }

  inline std::vector<Eigen::Vector3d> discretizeLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double resolution)
  {
    Eigen::Vector3d line = p2 - p1;
    double length = line.norm();
    int seg_num = ceil(length / resolution);

    std::vector<Eigen::Vector3d> new_line;
    if (seg_num <= 0) {
      return new_line;
    }

    for (int i = 0; i <= seg_num; ++i)
    new_line.push_back(p1 + line * double(i) / double(seg_num));

    return new_line;
  }


  inline std::vector<Eigen::Vector3d> rediscretizePath(const std::vector<Eigen::Vector3d>& path, double resolution)
  {
    std::vector<Eigen::Vector3d> new_path, segment;

    if (path.size() < 2) {
      ROS_ERROR("what path? ");
      return new_path;
    }

    // for (size_t i = 0; i < path.size() - 1; ++i) {
    //   segment = discretizeLine(path[i], path[i + 1], resolution);
    //
    //   if (segment.size() < 1) continue;
    //
    //   new_path.insert(new_path.end(), segment.begin(), segment.end());
    //   if (i != path.size() - 2) new_path.pop_back();
    // }
    // return new_path;

    int N = std::round(Length(path)/resolution);
    new_path = discretizePath(path, N);

    return new_path;
  }

  // version where the index vector is returned as well to show where the old path points are located.
  inline std::vector<Eigen::Vector3d> rediscretizePath(const std::vector<Eigen::Vector3d>& path, double resolution, std::vector<int>& idx_vec)
  {
    std::vector<Eigen::Vector3d> new_path;
    idx_vec.clear();
    int N = std::round(Length(path)/resolution);
    new_path = discretizePath(path, N, idx_vec);
    return new_path;

  }

  inline std::vector<Eigen::Vector3d> rediscretizePath(const std::vector<Eigen::Vector3d>& path)
  {
    return rediscretizePath(path, 0.1);
  }

  // find point along path

  inline std::tuple<int, planner::FlatState> findClosestPointAlongPath(const std::vector<planner::FlatState>& path, const Eigen::Vector3d& pos)
  {
    int idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    planner::FlatState closest_reference = path.front();

    // special conditions
    if (path.size() == 0)
        throw std::invalid_argument("[path_utils::findClosestPointAlongPath] path vector is empty!");
    if (path.size() == 1) return {-1, closest_reference};

    for (size_t i = 0; i < path.size(); i++)
    {
      double dist = (pos - path[i].pos).norm();

      if (dist < min_dist)
      {
        min_dist = dist;
        closest_reference = path[i];
        idx = i;
      }
    }

    return {idx, closest_reference};
  }

  inline std::tuple<int, Eigen::Vector3d> findClosestPointAlongPath(const std::vector<Eigen::Vector3d>& path, const Eigen::Vector3d& pos)
  {
    int idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    Eigen::Vector3d closest_reference = path.front();
    if (path.size() == 0)
      throw std::invalid_argument("[path_utils::findClosestPointAlongPath] path vector is empty!");
    if (path.size() == 1) return {-1, closest_reference};

    for (size_t i = 0; i < path.size(); i++)
    {
      double dist = (pos - path[i]).norm();

      if (dist < min_dist)
      {
        min_dist = dist;
        closest_reference = path[i];
        idx = i;
      }
    }

    return {idx, closest_reference};
  }

  inline std::tuple<int, planner::FlatState> findPointAtTime(const std::vector<planner::FlatState>& path, double t)
  {
    int idx = -1;
    planner::FlatState closest_reference = path.front();

    if (path.size() == 0)
      throw std::invalid_argument("[path_utils::findPointAtTime] path vector is empty!");
    if (path.size() == 1) return {idx, closest_reference};

    // Assuming uniform discretization
    double dt = path[1].t - path[0].t;
    double half_dt = dt*0.5;

    for (size_t i = 0; i < path.size(); i++)
    {
      // terminate if its close to half of the point
      if (path[i].t >= t || std::abs(path[i].t - t) < half_dt)
      {
        closest_reference = path[i];
        idx = i;
        break;
      }
    }

    return {idx, closest_reference};
  }

  // get segments of path

  inline std::tuple<double, std::vector<planner::FlatState>> getSegmentWithLength(const std::vector<planner::FlatState>& path, const Eigen::Vector3d& start_pos, double length)
  {
    if (path.size() == 0)
      throw std::invalid_argument("[path_utils::getSegmentWithLength] path vector is empty!");

    // initialize
    std::vector<planner::FlatState> segment;

    // find starting state
    auto [idx, start_state] = findClosestPointAlongPath(path, start_pos);
    segment.push_back(start_state);

    // check for special conditions
    if (length == 0) return {0.0, segment}; // if length is zero
    if (idx == path.size()-1) return {0.0, segment}; // if the starting point is beyond the trajectory

    // loop through from starting state and find the distance, or til the end of the path
    double dist = 0;
    for (int i = idx+1; i < (int)path.size(); i++)
    {
      double ds = (path[i].pos - path[i-1].pos).norm();
      segment.push_back(path[i]);
      dist += ds;
      if (dist >= length) break;
    }

    // duration is the time difference between the last element and the initial state
    double duration = segment.back().t - start_state.t;
    return {duration, segment};
  }


  // Extract a specific segment from the path with a fixed duration.

  inline std::tuple<double, std::vector<planner::FlatState>> getSegmentWithDuration(const std::vector<planner::FlatState>& path, const Eigen::Vector3d& start_pos, double duration)
  {
    if (path.size() == 0)
      throw std::invalid_argument("[path_utils::getSegmentWithDuration] path vector is empty!");

    // initialize
    std::vector<planner::FlatState> segment;

    // find starting state
    auto [idx, start_state] = findClosestPointAlongPath(path, start_pos);
    segment.push_back(start_state);

    // Check for special conditions
    if (duration == 0) return {0.0, segment}; // if the duration is zero
    if (idx == path.size()-1) return {0.0, segment}; // if the starting point is beyond the trajectory

    // loop through from starting state and find the duration, or til the end of the path
    for (int i = idx+1; i < (int)path.size(); i++)
    {
      segment.push_back(path[i]);
      double dt = std::abs(path[i].t - start_state.t);
      if (dt >= duration) break;
    }

    // T is the time difference between the last element and the initial state
    double T = segment.back().t - start_state.t;
    return {T, segment};
  }

  // Extract a fixed segment from a fixed duratin.

  inline std::tuple<double, std::vector<planner::FlatState>> getSegmentWithDuration(const std::vector<planner::FlatState>& path, double start_time, double duration)
  {

    // Check for special conditions
    if (path.size() == 0)
      throw std::invalid_argument("[path_utils::getSegmentWithDuration] path vector is empty!");

    // initialize
    std::vector<planner::FlatState> segment;
    if (duration == 0) return {0.0, segment}; // if the duration is zero

    auto [idx, start_state] = findPointAtTime(path, start_time);
    segment.push_back(start_state);

    // loop through from starting state and find the duration, or til the end of the path
    for (int i = idx+1; i < (int)path.size(); i++)
    {
      double dt = std::abs(path[i].t - start_time);
      if (dt >= duration) break;
    }

    // T is the time difference between the last element and the initial state
    double T = segment.back().t - start_state.t;
    return {T, segment};
  }


  // conversions

  inline std::vector<Eigen::Vector3d> convertStateToEigenPos(const std::vector<planner::FlatState>& path)
  {
    std::vector<Eigen::Vector3d> eigen_pos;
    for (auto & state : path)
    {
      eigen_pos.push_back(state.pos);
    }
    return eigen_pos;
  }

  inline std::vector<Eigen::Vector3d> convertStateToEigenVel(const std::vector<planner::FlatState>& path)
  {
    std::vector<Eigen::Vector3d> eigen_vel;
    for (auto & state : path)
    {
      eigen_vel.push_back(state.vel);
    }
    return eigen_vel;
  }

  inline std::vector<Eigen::Vector3d> convertStateToEigenAcc(const std::vector<planner::FlatState>& path)
  {
    std::vector<Eigen::Vector3d> eigen_acc;
    for (auto & state : path)
    {
      eigen_acc.push_back(state.acc);
    }
    return eigen_acc;
  }

  inline void print(const std::vector<planner::FlatState>& path, std::string str="", bool vel = false, bool acc = false, bool jerk = false, bool yaw = false)
  {
    printf("%s(%d):\n", str.c_str(), path.size());
    for (auto& state : path)
    {
      printf("t: %.2f pos [%.2f, %.2f, %.2f]", state.t, state.pos(0), state.pos(1), state.pos(2));
      if (vel)
        printf(" vel [%.2f, %.2f, %.2f]", state.vel(0), state.vel(1), state.vel(2));
      if (acc)
        printf(" acc [%.2f, %.2f, %.2f]", state.acc(0), state.acc(1), state.acc(2));
      if (jerk)
        printf(" jerk [%.2f, %.2f, %.2f]", state.jerk(0), state.jerk(1), state.jerk(2));
      if (yaw)
        printf(" yaw [%.2f]", state.yaw);
      printf("\n");
    }
  }


  inline void print(const std::vector<Eigen::Vector3d>& path, std::string str="")
  {
    printf("%s(%d):\n", str.c_str(), path.size());
    for (auto& state : path)
    {
      printf("pos [%.2f, %.2f, %.2f]\n", state(0), state(1), state(2));
    }
  }

  inline std::vector<double> getKnottimesFromSegmentTimes(const std::vector<double>& segment_times, double start_time = 0.0)
  {
    if (segment_times.size() == 0)
      throw std::invalid_argument("[path_utils::getKnottimesFromSegmentTimes] segment times contains zero elements!");

    std::vector<double> knottimes;
    knottimes.push_back(start_time);
    for (size_t i = 0; i < segment_times.size(); i++)
    {
      knottimes.push_back(knottimes[i]+segment_times[i]);
    }
    return knottimes;
  }

  inline std::vector<double> getSegmentTimesFromKnottimes(const std::vector<double>& knottimes)
  {
    if (knottimes.size() < 2)
      throw std::invalid_argument("[path_utils::getSegmentTimesFromKnottimes] knottimes has less than 2 values! Cannot compute durations.");

    std::vector<double> segment_times;
    for (size_t i = 1; i < knottimes.size(); i++)
    {
      segment_times.push_back(knottimes[i]-knottimes[i-1]);
    }
    return segment_times;
  }

}
