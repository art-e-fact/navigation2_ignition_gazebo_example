FROM public.ecr.aws/artefacts/nav2:jazzy-harmonic

WORKDIR /ws

# Install only the external packages first to speed up rebuilds
COPY deps.repos /tmp
RUN apt update && apt install -y python3-vcstool && rm -rf /var/lib/apt/lists/*
RUN mkdir src && vcs import --input /tmp/deps.repos src
RUN apt update -y && rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/jazzy/setup.bash --extend && colcon build --symlink-install

COPY . /ws
RUN rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/jazzy/setup.bash --extend && colcon build --symlink-install

RUN pip install -r src/sam_bot_nav2_gz/requirements.txt
CMD source /ws/install/setup.bash && artefacts run $ARTEFACTS_JOB_NAME
