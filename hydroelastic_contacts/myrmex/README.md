# Simulation of a Myrmex Tactile Sensor Array

* Author: David P. Leins <dleins@techfak.de>
* License: BSD 3-Clause License

## Prerequisites

- MuJoCo 2.3.6
- Drake (tested with 1.12.0 - 1.22.0)

## Installing and Running

1. Create catkin workspace (`mkdir -p mj_ros_demo_ws/src && cd mj_ros_demo_ws && catkin init`)
2. Clone the demo repository into the workspace with `git clone https://github.com/ubi-agni/mujoco_ros_demos src/mujoco_ros_demos`
3. Download ros package dependencies into catkin workspace src folder with `vcs import src < src/mujoco_ros_demos/hydroelastic_contacts/myrmex/baro.deps`
4. Install MuJoCo and Drake by extracting the tar container and setting the following environment variables:
```
export MUJOCO_DIR=PATH_TO_EXTRACTED_MUJOCO_DIR
export DRAKE_DIR=PATH_TO_EXTRACTED_DRAKE_DIR

export LD_LIBRARY_PATH=$MUJOCO_DIR/lib:$DRAKE_DIR/lib:$LD_LIBRARY_PATH
export CPATH=$MUJOCO_DIR/include:$CPATH
export LIBRARY_PATH=$MUJOCO_HOME/lib:$LIBRARY_PATH

```
5. Build catkin workspace with `catkin b mujoco_contact_surface_sensors` and source the devel workspace.
6. Run `roslaunch contact_surface_sensors myrmex_sensor.launch`.

sensor readings can be fetched on the `/tactile_module_16x16_v2` topic, e.g. in the terminal with `rostopic echo /tactile_module_16x16_v2`.

as additional launchfile argument you can provide `object:=` and set it to one of `box` (box primitive), `plate` (more complex plate mesh), or `spot` (spot the cow, very complex mesh).

The object spawning additionally to the sensor can be dragged around by double-clicking it to select it and then move the mouse around while holding ctrl + right mouse button.

In the simulation window, clicking `t` will toggle transparency of bodies, which makes inspection of the collisions easier. Pressing `w` toggles wireframe mode, making the rendered contact surface better visible.
In the top left corner the realtime factor is displayed. In case it shows 100%, you are running at realtime. You may set `realtime:=-1` to see how fast the current configuration can run on your machine.