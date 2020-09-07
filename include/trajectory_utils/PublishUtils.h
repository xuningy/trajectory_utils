#pragma once

#include <vector>
#include <ros/ros.h>

#include <control_arch/utils/state_t.h>
#include <control_arch/Polynomial.h>
#include <control_arch/MultiPolynomial.h>
#include <control_arch/Waypoints.h>
#include <control_arch/trajectory/Waypoints.h>
#include <geometry_utils/GeometryUtils.h>
#include <forward_arc_primitives/ForwardArcMotionPrimitives.h>

namespace publisher_utils
{
  inline void publishWaypoints(const std::vector<state_t>& traj_wpts, const ros::Time& publish_time, float traj_duration, ros::Publisher& wpts_pub)
  {
    if (traj_wpts.size() == 0) return;

    control_arch::Waypoints msg;
    Waypoints traj(traj_wpts);
    traj.toMessage(msg);

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";

    msg.trajectory_options.scheduling_mode =
      control_arch::TrajectoryOptions::SCHEDULE_AT_TIME;
    msg.trajectory_options.enable_on_ramp = false;
    msg.trajectory_options.required_flag = "teleop";

    msg.start_time = publish_time;

    wpts_pub.publish(msg);

    std::cout << "Publishing trajectory at time " << publish_time.toSec() << std::endl;
  }

  inline void publishPolynomial(const ros::Time& publish_time, const planner::ForwardArcMotionPrimitives& mp, ros::Publisher& poly_pub)
  {
    control_arch::Polynomial msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";

    msg.trajectory_options.scheduling_mode =
      control_arch::TrajectoryOptions::SCHEDULE_AT_TIME;
    msg.trajectory_options.enable_on_ramp = false;
    msg.trajectory_options.enable_off_ramp = false;
    msg.trajectory_options.required_flag = "teleop";

    size_t N = mp.coeff_w_[0].size();
    msg.polynomial.x.assign(mp.coeff_w_[0].data(),
                            mp.coeff_w_[0].data() + N);
    msg.polynomial.y.assign(mp.coeff_w_[1].data(),
                            mp.coeff_w_[1].data() + N);
    msg.polynomial.z.assign(mp.coeff_w_[2].data(),
                            mp.coeff_w_[2].data() + N);

    msg.polynomial.yaw = { mp.coeff_w_[3][0], mp.omega() };

    msg.polynomial.duration = ros::Duration(mp.duration());
    msg.polynomial.start_time_offset = 0.0;
    msg.polynomial.start_time = publish_time;

    poly_pub.publish(msg);
  }

  inline void publishZeroTrajectory(
    const state_t& ref_state, const ros::Time& ref_time, double duration, ros::Publisher& poly_pub)
  {
    geometry_utils::Vec4 zero_input = geometry_utils::Vec4(0.0, 0.0, 0.0, 0.0);
    planner::ForwardArcMotionPrimitives mp(ref_state, zero_input, duration);

    publishPolynomial(ref_time, mp, poly_pub);
    return;
  }
}
