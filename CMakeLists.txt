cmake_minimum_required(VERSION 2.8.3)
project(regular_kalman_filter)

find_package(catkin REQUIRED
  rospy
  tf
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   scripts/regular_kalman_filter_estimate.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
   scripts/plot_kalman_filter_estimate.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
