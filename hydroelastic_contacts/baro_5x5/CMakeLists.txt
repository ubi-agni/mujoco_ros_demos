cmake_minimum_required(VERSION 3.0.2)

project(baro_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  mujoco_ros
)

catkin_package(
 CATKIN_DEPENDS
  mujoco_ros
  mujoco_contact_surface_sensors
  mujoco_mocap
  mujoco_ros_sensors
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY assets 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)