# Moving the Franka Emika Panda with ROS Control

* Author: David P. Leins <dleins@techfak.de>
* License: BSD 3-Clause License

## Prerequisites

- MuJoCo (tested with 2.2.0 - 2.3.0)

## Installing

1. Create catkin workspace (`mkdir -p mj_ros_demo_ws/src && cd mj_ros_demo_ws && catkin init`)
2. Clone the demo repository into the workspace with `git clone https://github.com/ubi-agni/mujoco_ros_demos src/mujoco_ros_demos`
3. Download ros package dependencies into catkin workspace src folder with `vcs import > src/mujoco_ros_demos/panda_ros_control/panda.deps src`
4. Build catkin workspace with `catkin b mujoco_panda_waving` and source the devel workspace.
5. Run `roslaunch mujoco_panda_waving panda_auto_waving.launch use_sim_time:=true`.

Play around with the launchfile arguments, if you like.
