cmake_minimum_required(VERSION 3.0.2)
project(move_it_api)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    message_generation
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
# INCLUDE_DIRS include
# #  LIBRARIES move_it_api
# #  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
# #  DEPENDS system_lib
# )

add_service_files(
  FILES
  pose_server.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${THIS_PACKAGE_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIR} 
  ${EIGEN3_INCLUDE_DIRS}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# add_executable(${PROJECT_NAME}_server src/pose_server.cpp)
add_executable(${PROJECT_NAME}_client src/pose_client.cpp)
# add_dependencies(${PROJECT_NAME}_server )
# target_link_libraries(${PROJECT_NAME}_server ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}_client ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
# target_link_libraries(${PROJECT_NAME}_client ${catkin_LIBRARIES})
# install(TARGETS ${PROJECT_NAME}_server DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# install(TARGETS ${PROJECT_NAME}_client DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
