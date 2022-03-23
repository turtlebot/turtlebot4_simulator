# Turtlebot4 Simulator

Turtlebot4 Simulation using Ignition Gazebo.

## Prerequisites

### ROS2 Galactic

Install from [debian](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html).

### ROS2 Dev Tools

```bash
sudo apt install -y \
python3-colcon-common-extensions \
python3-rosdep \
python3-vcstool
```

### Ignition Edifice

```bash
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ignition-edifice
```

**Note**: `ign_ros2_control` is released but not yet sync in the Galactic stable repository. Galactic sync
happens perodically. Meanwhile you can try to use the testing repository:

#### ROS2 Testing packages

Change your ROS2 package source to `ros2-testing` to get the latest updates.

 - Edit `/etc/apt/sources.list.d/ros2.list`
 - Change `http://packages.ros.org/ros2/ubuntu focal main` to `http://packages.ros.org/ros2-testing/ubuntu focal main`
 - Run `sudo apt update`

## Build

- Create a workspace if you don't already have one:

```bash
mkdir -p ~/turtlebot4_ws/src
```

- Clone this repository into the src directory from above

```bash
cd ~/turtlebot4_ws/src
git clone https://github.com/turtlebot/turtlebot4_simulator.git
```

- Use `vcs` to clone additional dependencies into the workspace:

```bash
vcs import ~/turtlebot4_ws/src/ < ~/turtlebot4_ws/src/turtlebot4_simulator/dependencies.repos
```

- Navigate to the workspace and install ROS 2 dependencies with:

```bash
cd ~/turtlebot4_ws
rosdep install --from-path src -yi
```

- Build the workspace:

```bash
source /opt/ros/galactic/setup.bash
export IGNITION_VERSION=edifice
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

## SLAM
`ros2 launch turtlebot4_ignition_bringup ignition.launch.py slam:=true rviz:=true`

## Nav2

Run simulation with SLAM

In second terminal:
`ros2 launch turtlebot4_ignition_bringup nav2.launch.py`
