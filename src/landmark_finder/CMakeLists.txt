cmake_minimum_required(VERSION 3.0.2)
project(landmark_finder)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES landmark_finder
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS
   src/ar_to_map_02.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )
catkin_install_python(PROGRAMS
   src/landmark_finder.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )