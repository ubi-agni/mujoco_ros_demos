cmake_minimum_required(VERSION 2.8.3)
project(mujoco_panda_waving)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS
  scripts/movement_with_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
