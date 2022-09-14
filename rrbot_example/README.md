# MuJoCo/_ROS RRBot Example

* Author: David P. Leins <dleins@techfak.de>
* License: GNU General Public License, version 3 (GPL-3.0)

## Prerequisites

- MuJoCo version 2.2.1
- MuJoCo 2.2.1 python bindings (`pip install mujoco==2.2.1`)

## Installing

1. Create catkin workspace (`mkdir -p mj_ros_demo_ws/src && cd mj_ros_demo_ws && catkin init`)
2. Clone the demo repository into the workspace with `git clone https://github.com/DavidPL1/mujoco_ros_demos src/mujoco_ros_demos`
3. Download ros package dependencies into catkin workspace src folder with `vcs import > src/mujoco_ros_demos/rrbot_example/rrbot.deps src`
4. Build catkin workspace with `catkin b rrbot_mujoco` and source the devel space
5. Run `roslaunch rrbot_mujoco rrbot_launch`

by streaming to the joint position controller command topics (check the boxes in the spawned rqt window for a sine signal) the two joints of the robot can be controlled. 

### rrbot\_launch
Set `use_urdf2mjcf:=true` to generate the MuJoCo xml directly from the URDF with [urdf2mjcf](https://github.com/balandbal/urdf2mjcf).
Set `rviz:=true` to start an rviz visualisation. 