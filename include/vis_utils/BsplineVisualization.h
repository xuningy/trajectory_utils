#pragma once

#include <vector>
#include <ros/ros.h>

#include <bspline/non_uniform_bspline.h>
#include <vis_utils/BaseVisualization.h>

namespace vis_utils
{

inline void visualizeBSpline(fast_planner::NonUniformBspline& bspline, const std_msgs::ColorRGBA& color, const ros::Publisher& pub, bool show_ctrl_pts, double size_ctrlpts, const std_msgs::ColorRGBA& color_ctrlpts, int id1, int id_ctrlpts, const ros::Publisher& pub_ctrlpts)
{
  if (bspline.getControlPoint().size() == 0) return;

  vector<Eigen::Vector3d> traj_pts;
  double tm, tmp;
  bspline.getTimeSpan(tm, tmp);

  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::Vector3d pt = bspline.evaluateDeBoor(t);
    traj_pts.push_back(pt);
  }
  visualizeLine(traj_pts, color, id1, pub);

  // draw the control point
  if (!show_ctrl_pts) return;

  Eigen::MatrixXd ctrl_pts = bspline.getControlPoint();
  vector<Eigen::Vector3d> ctp;

  for (int i = 0; i < int(ctrl_pts.rows()); ++i) {
    Eigen::Vector3d pt = ctrl_pts.row(i).transpose();
    ctp.push_back(pt);
  }

  visualizeSphereList(ctp, size_ctrlpts, color_ctrlpts, id_ctrlpts, pub_ctrlpts);
}

}
