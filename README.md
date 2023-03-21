# Navigation2 (Ignition) Gazebo Example
Minimal example ROS2 project to use Navigation2 with (Ignition) Gazebo. Based on the official Gazebo Classic example: [tutorial](https://navigation.ros.org/setup_guides/index.html), [code](https://github.com/ros-planning/navigation2_tutorials/tree/master/sam_bot_description)

## Setup
```
#install Nav2 dependencies 
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## Set navigation goals with RViz
TODO

## Visualize logs with Rerun
TODO

## Run tests locally


## Run tests with Artefacts
```
# Locally
artefacts run tests

# Remotely
artefacts run-remote tests --description "Test Navigation2"

# Locally with Docker
docker build --build-arg MAKEFLAGS=-j5 -t nav2-gz .
docker run --rm -e ARTEFACTS_JOB_NAME=tests -e ARTEFACTS_KEY=<API_KEY> nav2-gz
```