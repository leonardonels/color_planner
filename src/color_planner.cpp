#include "color_planner/color_planner.hpp"
#include <algorithm>
#include <cmath>

namespace color_planner
{

ColorPlanner::ColorPlanner() : Node("color_planner")
{
  frame_id_ = "map";

  marker_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
    "/cones", rclcpp::SensorDataQoS(),
    std::bind(&ColorPlanner::marker_callback, this, std::placeholders::_1));

  left_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/left_path", 10);
  right_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/right_path", 10);
  center_line_pub_ = this->create_publisher<nav_msgs::msg::Path>("/center_line", 10);
}

void ColorPlanner::marker_callback(const visualization_msgs::msg::Marker::SharedPtr msg)
{
  if (msg->points.size() != msg->colors.size())
  {
    RCLCPP_WARN(this->get_logger(), "Mismatched number of points and colors");
    return;
  }
  process_cones(msg->points, msg->colors);
}

void ColorPlanner::process_cones(const std::vector<geometry_msgs::msg::Point> &points,
                                 const std::vector<std_msgs::msg::ColorRGBA> &colors)
{
  std::vector<geometry_msgs::msg::Point> left_cones, right_cones;

  for (size_t i = 0; i < points.size(); ++i)
  {
    const auto &color = colors[i];
    if (color.r > 0.9 && color.g > 0.9 && color.b < 0.1) // Yellow
    {
      right_cones.push_back(points[i]);
    }
    else if (color.b > 0.9 && color.r < 0.1 && color.g < 0.1) // Blue
    {
      left_cones.push_back(points[i]);
    }
  }

  auto left_path = create_path_from_cones(left_cones);
  auto right_path = create_path_from_cones(right_cones);
  auto center_path = compute_center_line(left_path, right_path);

  left_path_pub_->publish(left_path);
  right_path_pub_->publish(right_path);
  center_line_pub_->publish(center_path);
}

nav_msgs::msg::Path ColorPlanner::create_path_from_cones(const std::vector<geometry_msgs::msg::Point> &cones)
{
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = frame_id_;

  std::vector<geometry_msgs::msg::Point> sorted = cones;
  std::sort(sorted.begin(), sorted.end(), [](const auto &a, const auto &b) {
    return a.x < b.x; // Sort by X for simplicity
  });

  for (const auto &pt : sorted)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = pt;
    pose.pose.orientation.w = 1.0; // neutral orientation
    path.poses.push_back(pose);
  }
  return path;
}

nav_msgs::msg::Path ColorPlanner::compute_center_line(const nav_msgs::msg::Path &left_path, const nav_msgs::msg::Path &right_path)
{
  nav_msgs::msg::Path center_path;
  center_path.header.stamp = this->now();
  center_path.header.frame_id = frame_id_;

  size_t n = std::min(left_path.poses.size(), right_path.poses.size());
  for (size_t i = 0; i < n; ++i)
  {
    const auto &left_pt = left_path.poses[i].pose.position;
    const auto &right_pt = right_path.poses[i].pose.position;

    geometry_msgs::msg::PoseStamped center_pose;
    center_pose.header = center_path.header;
    center_pose.pose.position.x = 0.5 * (left_pt.x + right_pt.x);
    center_pose.pose.position.y = 0.5 * (left_pt.y + right_pt.y);
    center_pose.pose.position.z = 0.5 * (left_pt.z + right_pt.z);
    center_pose.pose.orientation.w = 1.0;

    center_path.poses.push_back(center_pose);
  }

  return center_path;
}

} // namespace color_planner
