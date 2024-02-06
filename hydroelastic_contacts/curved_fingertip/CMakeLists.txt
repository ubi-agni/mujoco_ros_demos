cmake_minimum_required(VERSION 3.0.2)

project(curved_sensor_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  mujoco_ros
)

catkin_package(
 CATKIN_DEPENDS
  mujoco_ros
  mujoco_contact_surface_sensors
  mujoco_ros_mocap
  mujoco_ros_sensors
)

install(DIRECTORY launch config assets
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)