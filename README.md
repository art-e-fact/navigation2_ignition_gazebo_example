
# Navigation2 (Ignition) Gazebo Example


[![artefacts_ci](https://app.artefacts.com/api/artefacts/navigation2-ignition-example/badges/nav2.png?branch=main)](https://app.artefacts.com/artefacts/navigation2-ignition-example)

Minimal example ROS2 project to use Navigation2 with (Ignition) Gazebo. Based on the official Gazebo Classic example: [tutorial](https://navigation.ros.org/setup_guides/index.html), [code](https://github.com/ros-planning/navigation2_tutorials/tree/master/sam_bot_description)

[gz-nav2-tb3.webm](https://user-images.githubusercontent.com/2298371/226628768-818a7c3f-e5e1-49c6-b819-112c2cfa668b.webm)


## Requirements
 - ROS 2 Humble
 - Gazebo Fortress
 - [Navigation 2](https://navigation.ros.org/build_instructions/index.html#install)

## Setup and build
```
# Install Nav2 dependencies
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Import source dependencies
pip3 install vcstool
vcs import --input deps.repos src

# Install rosrep dependencies
rosdep install -y -r -i  --from-paths . 

# Make sure ROS2 is sourced (assuming bash, please replace extension as needed)
source /opt/ros/humble/setup.bash

# Build
colcon build

# Make sure the app is sourced (assuming bash, please replace extension as needed)
source install/setup.bash
```

## Run examples
```
# Launch Gazebo, RViz, and Navigation2
ros2 launch sam_bot_nav2_gz complete_navigation.launch.py

# Set goal poses in RViz or run a navigation example:
ros2 run sam_bot_nav2_gz follow_waypoints.py
ros2 run sam_bot_nav2_gz reach_goal.py
```

## Run tests with **launch_testing**

You will need to pip install the `artefacts-toolkit` package to run the tests. 

```
# Start one of the tests with
# if using a virtualenv you may need to add the path to the python packages to make it available in the ROS2 environment:
# export PYTHONPATH=$PYTHONPATH:$HOME/.pyenv/versions/[venv-name]/lib/python3.10/site-packages
launch_test src/sam_bot_nav2_gz/test/test_bringup.launch.py
launch_test src/sam_bot_nav2_gz/test/test_reach_goal.launch.py
launch_test src/sam_bot_nav2_gz/test/test_follow_waypoints.launch.py
```



## Run tests with **Artefacts CI**
 1. Set up a new Artefacts CI project. Instructions: https://docs.artefacts.com/latest/
 2. Replace the project name in the `artefacts.yaml` with the name of your project


You will need to pip install the `artefacts-toolkit` and `artefacts-cli` packages to run the tests. 

```
# Run test locally
artefacts run all

# Run test remotely
artefacts run-remote all --description "Test Navigation2"

# Run test locally with Docker
docker build -t nav2-gz .
# ARTEFACTS_KEY can be generated at the project-settings page of the Artefacts CI dashboard
docker run --rm -e ARTEFACTS_JOB_NAME=all -e ARTEFACTS_KEY=${ARTEFACTS_KEY} nav2-gz
```

## Visualize navigation with **Rerun.io** (experimental)
For more info, see: https://www.rerun.io/docs/howto/ros2-nav-turtlebot
```
cd rerun
# create a virtual env and install dependencies
python3 -m venv rr-venv
source rr-venv/bin/activate
pip install -r requirements
# Run rerun node
python run-rerun.py
```
![Screenshot from 2023-03-27 20-42-32](https://user-images.githubusercontent.com/2298371/228792085-66837913-32fe-4506-9624-673424328ea4.png)

