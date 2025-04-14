
# Navigation2 (Ignition) Gazebo Example


[![artefacts_ci](https://app.artefacts.com/api/artefacts/navigation2-ignition-example/badges/nav2.png?branch=main)](https://app.artefacts.com/artefacts/navigation2-ignition-example)

Minimal example for testing a Nav2 project with Gazebo Sim.

https://github.com/user-attachments/assets/05dcb944-e6c9-42af-bff3-862eba04e53e

## Requirements
 - ROS 2 Jazzy
 - Gazebo Harmonic
 - [Navigation 2](https://navigation.ros.org/build_instructions/index.html#install)

## Setup and build
```
# Install Nav2 dependencies
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# Install rosrep dependencies
rosdep install -y -r -i --from-paths . 

# Make sure ROS2 is sourced (assuming bash, please replace extension as needed)
source /opt/ros/jazzy/setup.bash

# Build
colcon build

# Make sure the app is sourced (assuming bash, please replace extension as needed)
source install/setup.bash
```

## Run example
```
ros2 run nav2_gz_testing example_waypoint_follower.py
```

## Run tests with **pytest** locally
First install the python dependencies. See the [Jazzy docs](https://docs.ros.org/en/jazzy/How-To-Guides/Using-Python-Packages.html#installing-via-a-virtual-environment) for instructions to set up Python virtual environments with ROS 2 Jazzy.
```sh
pip install -r src/nav2_gz_testing/requirements.txt
```
And run the tests with standard pytest
```
pytest src/nav2_gz_testing/test/test_follow_waypoints.py -s
```


## Run tests with **Artefacts CI**
 1. Set up a new Artefacts CI project. Instructions: https://docs.artefacts.com/latest/
 2. Replace the project name in the `artefacts.yaml` with the name of your project


You will need to pip install the `artefacts-toolkit` and `artefacts-cli` packages to run the tests. 

```
# Run test locally
artefacts run nav2

# Run test remotely
artefacts run-remote nav2 --description "Test Navigation2"
```
### Run test locally with Docker

1. Build container
```sh
docker/build.sh
```
2. Save `.env.sample` as `.env` and fill the missing variables
    - ARTEFACTS_KEY can be generated at the project-settings page of the [Artefacts CI dashboard](https://app.artefacts.com/)
3. Run the selected tests in docker
```sh
docker run --rm --env-file=.env -e ARTEFACTS_JOB_NAME=nav2 nav2-gz
```
 - You can change the `headless` setting to `False` in the `artefacts.yaml` to run the tests with GUI
