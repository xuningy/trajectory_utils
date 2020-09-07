#pragma once

#include <vector>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

#include <control_arch/utils/state_t.h>
#include <geometry_utils/GeometryUtilsROS.h>

namespace gr = geometry_utils::ros;

namespace vis_utils
{

inline Eigen::Vector4d red_(1.0, 0.0, 0.0, 1.0);
inline Eigen::Vector4d green_(0.0, 1.0, 0.0, 1.0);
inline Eigen::Vector4d blue_(0.0, 0.0, 1.0, 1.0);
inline Eigen::Vector4d yellow_(1.0, 1.0, 0.0, 1.0);
inline Eigen::Vector4d cyan_(0.0, 1.0, 1.0, 1.0);
inline Eigen::Vector4d magenta_(1.0, 0.0, 1.0, 1.0);
inline Eigen::Vector4d grey_(0.7, 0.7, 0.7, 0.7);
inline Eigen::Vector4d purple_(0.5, 0.0, 0.5, 0.5);

inline void visualizeSphereList(const std::vector<Eigen::Vector3d>& list, double resolution, const std_msgs::ColorRGBA& color, int id, const ros::Publisher& pub) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color = color;

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (size_t i = 0; i < list.size(); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  pub.publish(mk);
}

inline void visualizeSphereList(const std::vector<Eigen::Vector3d>& list, double resolution, const Eigen::Vector4d& color, int id, const ros::Publisher& pub)
{
  std_msgs::ColorRGBA color_msg;
  color_msg.r = color(0);
  color_msg.g = color(1);
  color_msg.b = color(2);
  color_msg.a = color(3);

  visualizeSphereList(list, resolution, color_msg, id, pub);
}

inline void visualizeLine(const std::vector<Eigen::Vector3d>& list, const std_msgs::ColorRGBA& color, int id, const ros::Publisher& pub)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::LINE_STRIP;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color = color;

  mk.scale.x = 0.1;
  mk.scale.y = 0.1;
  mk.scale.z = 0.1;

  geometry_msgs::Point pt;
  for (size_t i = 0; i < list.size(); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  pub.publish(mk);
}


inline void visualizeLine(const std::vector<Eigen::Vector3d>& list, const Eigen::Vector4d& color, int id, const ros::Publisher& pub)
{
  std_msgs::ColorRGBA color_msg;
  color_msg.r = color(0);
  color_msg.g = color(1);
  color_msg.b = color(2);
  color_msg.a = color(3);

  visualizeLine(list, color_msg, id, pub);
}


inline void visualizeLine(const std::vector<state_t>& states, const std_msgs::ColorRGBA& color, int id, const ros::Publisher& pub)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::LINE_STRIP;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color = color;

  mk.scale.x = 0.1;
  mk.scale.y = 0.1;
  mk.scale.z = 0.1;

  for (auto& state : states) {
    mk.points.push_back(gr::toPoint(state.pos));
  }
  pub.publish(mk);
}


inline void visualizeLine(const std::vector<state_t>& states, const Eigen::Vector4d& color, int id, const ros::Publisher& pub)
{
  std_msgs::ColorRGBA color_msg;
  color_msg.r = color(0);
  color_msg.g = color(1);
  color_msg.b = color(2);
  color_msg.a = color(3);
  visualizeLine(states, color_msg, id, pub);
}

}
