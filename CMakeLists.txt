cmake_minimum_required(VERSION 3.0.2)
project(path_following)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  rosbag
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_following_node src/path_following_node.cpp)
target_link_libraries(path_following_node ${catkin_LIBRARIES})
