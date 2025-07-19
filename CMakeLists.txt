cmake_minimum_required(VERSION 3.8)
project(color_planner)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
)

add_library(color_planner src/color_planner.cpp)
ament_target_dependencies(
  color_planner
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
)

add_executable(color_planner_node src/color_planner_node.cpp)
ament_target_dependencies(
  color_planner_node
  rclcpp
)

target_link_libraries(color_planner_node
  color_planner
)

install(TARGETS
  color_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
