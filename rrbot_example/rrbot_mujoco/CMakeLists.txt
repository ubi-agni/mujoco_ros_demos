cmake_minimum_required(VERSION 2.8.3)
project(rrbot_mujoco)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/xacro_pipe
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)