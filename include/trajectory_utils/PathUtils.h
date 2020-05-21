#pragma once

#include <Eigen/Eigen>
#include <vector>

namespace path_utils
{

  inline double Length(const std::vector<Eigen::Vector3d>& path)
  {
    double length = 0.0;
    if (path.size() < 2) return length;

    for (size_t i = 0; i < path.size() - 1; ++i) {
      length += (path[i + 1] - path[i]).norm();
    }
    return length;
  }

  inline std::vector<Eigen::Vector3d> discretizePath(const std::vector<Eigen::Vector3d>& path, int pt_num) {
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

  inline std::vector<Eigen::Vector3d> discretizeLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double resolution) {
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

    for (size_t i = 0; i < path.size() - 1; ++i) {
      segment = discretizeLine(path[i], path[i + 1], resolution);

      if (segment.size() < 1) continue;

      new_path.insert(new_path.end(), segment.begin(), segment.end());
      if (i != path.size() - 2) new_path.pop_back();
    }
    return new_path;
  }


  inline std::vector<Eigen::Vector3d> rediscretizePath(const std::vector<Eigen::Vector3d>& path)
  {
    return rediscretizePath(path, 0.1);
  }

}
