# navigation2_tutorials
Tutorial code referenced in https://navigation.ros.org/

```
#install Nav2 dependencies 
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## Run tests with Artefacts
```
# Locally
artefacts run tests

# Remotely
artefacts run-remote tests --description "Test Navigation2"

# Locally with Docker
docker build --build-arg MAKEFLAGS=-j8 -t nav2-gz .
docker run --rm -e ARTEFACTS_JOB_NAME=tests -e ARTEFACTS_KEY=<API_KEY> nav2-gz
```