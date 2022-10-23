cmake_minimum_required(VERSION 3.0.2)
project(custom_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  robot_state_publisher
  sensor_msgs
  geometry_msgs
  tf 
  tf2
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES custom_control
 CATKIN_DEPENDS roscpp geometry_msgs tf tf2 
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/joy_to_geometry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# Declare a C++ library#
# add_library(${PROJECT_NAME}
#   src/cartesian_motion_controller.cpp
#   include/cartesian_motion_controller/cartesian_motion_controller.h
#   include/cartesian_motion_controller/cartesian_motion_controller.hpp
# )

## Mark other files for installation (e.g. launch and bag files, etc.)


add_executable(remap src/remap.cpp)
target_link_libraries(remap ${catkin_LIBRARIES})


add_executable(cmc src/main.cpp src/complianceMovementController.cpp src/joystick.cpp src/jointTrajectoryController)
target_link_libraries(cmc ${catkin_LIBRARIES})


# add_executable(jtc  src/mainJQT.cpp src/jointTrajectoryController.cpp)
# target_link_libraries(jtc ${catkin_LIBRARIES})

# add_dependencies(cmc geometry_msgs)