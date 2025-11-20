cmake_minimum_required(VERSION 3.0.2)
project(my_robot_tracker)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  my_robot_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs my_robot_msgs
)

catkin_install_python(PROGRAMS
  nodes/pure_pursuit_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
