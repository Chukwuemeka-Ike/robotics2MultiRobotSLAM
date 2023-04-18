cmake_minimum_required(VERSION 3.0.2)
project(landmark_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## System dependencies a
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES landmark_navigation
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# add_executable(landmark_nav landmark_nav.py)
catkin_install_python(PROGRAMS
   src/landmark_nav.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
   src/clicked_point_sub.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS 
   src/test_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)