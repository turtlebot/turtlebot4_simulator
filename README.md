# Turtlebot4 Simulator

Turtlebot4 Simulation using Ignition Gazebo.

## Prerequisites

### Ignition Edifice

Follow the [Create3 Instructions](https://github.com/iRobotEducation/create3_sim/blob/main/README.md#prerequisites) to install Ignition Edifice. Installing from source is recommended.

## Build

- Create a workspace if you don't already have one:

```bash
mkdir -p ~/turtlebot4_ws/src
```

- Clone this repository into the src directory from above

```bash
cd ~/turtlebot4_ws/src
git clone git@github.com:turtlebot/turtlebot4_sim.git
```

- Use `vcs` to clone additional dependencies into the workspace:

```bash
vcs import ~/turtlebot4_ws/src/ < ~/turtlebot4_ws/src/turtlebot4_sim/dependencies.repos
```

- Clone turtlebot4 packages:

```bash
cd ~/turtlebot4_ws/src
git clone git@github.com:turtlebot/turtlebot4.git
git clone git@github.com:turtlebot/turtlebot4_desktop.git
```

- Navigate to the workspace and install ROS 2 dependencies with:

```bash
cd ~/turtlebot4_ws
rosdep install --from-path src -yi
```

- Build the workspace:

```bash
source /opt/ros/galactic/setup.bash
```

- If Ignition is installed from source, source the Ignition workspace:

```bash
source ~/ignition_ws/install/setup.bash
```

- Then:

```bash
colcon build --symlink-install
source install/local_setup.bash
```

# Running Simulation

## Default
`ros2 launch turtlebot4_ignition_bringup ignition.launch.py`

### Select Model
`ros2 launch turtlebot4_ignition_bringup ignition.launch.py model:=standard`

`ros2 launch turtlebot4_ignition_bringup ignition.launch.py model:=lite`

### Select World
`ros2 launch turtlebot4_ignition_bringup ignition.launch.py world:=depot.sdf`

`ros2 launch turtlebot4_ignition_bringup ignition.launch.py world:=maze.sdf`

## SLAM w/ LIDAR
`ros2 launch turtlebot4_ignition_bringup ignition.launch.py slam:=lidar rviz:=true`

## Nav2

Run simulation with SLAM

In second terminal:
`ros2 launch turtlebot4_ignition_bringup nav2.launch.py`
