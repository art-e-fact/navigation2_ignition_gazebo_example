FROM public.ecr.aws/artefacts/nav2:humble-fortress

WORKDIR /ws

# Install only the external packages first to speed up rebuilds
COPY deps.repos /tmp
RUN apt update && apt install -y python3-vcstool && rm -rf /var/lib/apt/lists/*
RUN mkdir src && vcs import --input /tmp/deps.repos src
RUN apt update -y && rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/humble/setup.bash --extend && colcon build --symlink-install

COPY . /ws
RUN apt update -y && apt install -y ros-humble-rosbag2-storage-mcap && rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/humble/setup.bash --extend && colcon build --symlink-install

RUN pip install -r src/sam_bot_nav2_gz/requirements.txt
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 launch sam_bot_nav2_gz complete_navigation.launch.py"]