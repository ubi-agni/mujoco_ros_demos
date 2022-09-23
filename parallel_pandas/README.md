# Starting Multiple instances of Franka Emika Panda in Parallel (Single ROS Core)

* Author: David P. Leins <dleins@techfak.de>
* License: BSD 3-Clause License

## Prerequisites

- MuJoCo version 2.2.2 (might also work with greater version)

## Installing

1. Create catkin workspace (`mkdir -p mj_ros_demo_ws/src && cd mj_ros_demo_ws && catkin init`)
2. Clone the demo repository into the workspace with `git clone https://github.com/DavidPL1/mujoco_ros_demos src/mujoco_ros_demos`
3. Download ros package dependencies into catkin workspace src folder with `vcs import > src/mujoco_ros_demos/panda_ros_control/panda.deps src`
4. Build catkin workspace with `catkin b mujoco_multi_env_demo` and source the devel workspace.
5. Run `roslaunch mujoco_multi_env_demo waving_pandas.launch`.


## Notes

If at least two environments (`num_instances`, default=2) are selected and `rviz` is set to true, MuJoCo will start showing a single moving panda robot. Additionally two rviz instances showing each a robot of another environment. Go ahead and perturb the currently active robot in the simulation window by double-clicking a part of the robot, then holding `ctrl` and the right mouse button, and drag the mouse around. You should now see one of the rviz robots moving according to what happens in your simulation view and the other should carry on following its trajectory unhindered.

Both robots are fully separated in enclosed ROS namespaces `env0` and `env1` (`env0` to `envN` for N environments). When disabling the trajectory publishers that make the robots move (with `publish_trajectory:=false**) you can send commands to steer the robots individually.

Note that currently parallel env modes starts the simulation in a paused state, no matter how the launch argument `unpause` is set.

*IMPORTANT*: When using an own `foo_bringup` launchfile for bootstrapping, everything that should be encapsulated in an environment should be called within a ```<group ns="$(arg ns)">``` tag. The bootstrapper node sets the namespace of each environment automatically when bootstrapping it.
