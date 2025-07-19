#ifndef COLOR_PLANNER__COLOR_PLANNER_HPP_
#define COLOR_PLANNER__COLOR_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <nav_msgs/msg/path.hpp>

namespace color_planner
{

class ColorPlanner : public rclcpp::Node
{
public:
  ColorPlanner();

private:
  void marker_callback(const visualization_msgs::msg::Marker::SharedPtr msg);
  void process_cones(const std::vector<geometry_msgs::msg::Point> &points,
                     const std::vector<std_msgs::msg::ColorRGBA> &colors);
  nav_msgs::msg::Path create_path_from_cones(const std::vector<geometry_msgs::msg::Point> &cones);
  nav_msgs::msg::Path compute_center_line(const nav_msgs::msg::Path &left_path,
                                          const nav_msgs::msg::Path &right_path);

  std::string frame_id_;

  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr left_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr right_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr center_line_pub_;
};

}  // namespace color_planner

#endif  // COLOR_PLANNER__COLOR_PLANNER_HPP_
