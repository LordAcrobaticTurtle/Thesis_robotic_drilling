cmake_minimum_required(VERSION 3.0.2)
project(utility)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(utility src/main.cpp)
target_link_libraries(utility ${catkin_LIBRARIES})
