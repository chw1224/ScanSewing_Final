cmake_minimum_required(VERSION 2.8.3)
project(scan_sewing_vision)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES scan_sewing_vision
#  CATKIN_DEPENDS  rospy
#  DEPENDS system_lib
)

catkin_install_python ( PROGRAMS scripts/vision_node.py 
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)