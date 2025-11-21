cmake_minimum_required(VERSION 3.0.2)
project(my_robot_logger)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  tf
  rospkg
)

catkin_package()

catkin_install_python(PROGRAMS
  nodes/ground_truth_logger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
