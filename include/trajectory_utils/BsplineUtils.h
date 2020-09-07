#pragma once

#include <Eigen/Eigen>
#include <bspline/non_uniform_bspline.h>

namespace bspline_utils
{
  fast_planner::NonUniformBspline getBspline(const Eigen::Matrix<double, Eigen::Dynamic, 3>&  path, double dt)
  {
    // HACK
    constexpr double max_vel_ = 1.5;
    constexpr double max_acc_ = 3.0;
    fast_planner::NonUniformBspline bspline = fast_planner::NonUniformBspline(path, 3, dt);
    bspline.setPhysicalLimits(max_vel_, max_acc_);
    bool feasible = bspline.checkFeasibility(false);

    int iter = 0;
    while (!feasible && iter < 10) {
      ROS_INFO("[bspline_utils] Bspline not feasible, reallocating time for attempt %d...", iter+1);
      feasible = bspline.reallocateTime();
      iter++;
    }

    return bspline;
  }


  inline double pathLength(const vector<Eigen::Vector3d>& path)
  {
    double length = 0.0;
    if (path.size() < 2) return length;

    for (int i = 0; i < path.size() - 1; ++i) {
      length += (path[i + 1] - path[i]).norm();
    }
    return length;
  }



}
