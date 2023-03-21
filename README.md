
# Navigation2 (Ignition) Gazebo Example
Minimal example ROS2 project to use Navigation2 with (Ignition) Gazebo. Based on the official Gazebo Classic example: [tutorial](https://navigation.ros.org/setup_guides/index.html), [code](https://github.com/ros-planning/navigation2_tutorials/tree/master/sam_bot_description)

[gz-nav2-tb3.webm](https://user-images.githubusercontent.com/2298371/226628768-818a7c3f-e5e1-49c6-b819-112c2cfa668b.webm)

## Setup and build
```
# Install Nav2 dependencies 
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Import source dependencies
vcs import --input create3_3.repos src

# Install rosrep dependencies
rosdep install -y -r -i  --from-paths . 

# Make sure ROS2 is sourced
source /opt/ros/humble/setup.bash

# Build
colcon build --symlink-install
```

## Run examples
```
# Launch Gazebo, RViz, and Navigation2
ros2 launch sam_bot_nav2_gz complete_navigation.launch.py

# Set goal poses in RViz or run a navigation example:
ros2 run sam_bot_nav2_gz follow_waypoints.py
ros2 run sam_bot_nav2_gz reach_goal.py
```


## Run tests with Artefacts
# TODO add instructions to create a new Artefacts project
```
# Locally
artefacts run tests

# Remotely
artefacts run-remote tests --description "Test Navigation2"

# Locally with Docker
docker build --build-arg MAKEFLAGS=-j5 -t nav2-gz .
docker run --rm -e ARTEFACTS_JOB_NAME=tests -e ARTEFACTS_KEY=<API_KEY> nav2-gz
```
