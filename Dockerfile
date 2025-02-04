FROM public.ecr.aws/artefacts/ros2:humble-fortress

WORKDIR /ws

RUN apt update && apt install -y ros-humble-navigation2 ros-humble-nav2-bringup && rm -rf /var/lib/apt/lists/*

# Install only the external packages first to speed up rebuilds
COPY deps.repos /tmp
RUN apt update && apt install -y python3-vcstool && rm -rf /var/lib/apt/lists/*
RUN mkdir src && vcs import --input /tmp/deps.repos src
RUN apt update -y && rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/humble/setup.bash --extend && colcon build --symlink-install --executor sequential

COPY . /ws
RUN apt update -y && rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/humble/setup.bash --extend && colcon build --symlink-install --executor sequential

CMD source /ws/install/setup.bash && artefacts run $ARTEFACTS_JOB_NAME
