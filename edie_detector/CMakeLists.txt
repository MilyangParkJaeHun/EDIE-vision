cmake_minimum_required(VERSION 3.0.2)
project(edie_detector)

find_package(catkin REQUIRED COMPONENTS
  edie_msgs
  rospy
  std_msgs
)

catkin_package(CATKIN_DEPENDS edie_msgs rospy std_msgs)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/edie_detector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)