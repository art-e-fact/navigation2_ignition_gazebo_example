FROM public.ecr.aws/artefacts/nav2:jazzy-harmonic

WORKDIR /ws

COPY . /ws

# Install dependencies
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN pip install -r src/nav2_gz_testing/requirements.txt

RUN source /opt/ros/jazzy/setup.bash --extend && colcon build --symlink-install


CMD source /ws/install/setup.bash && artefacts run $ARTEFACTS_JOB_NAME
