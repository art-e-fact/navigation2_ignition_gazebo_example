FROM public.ecr.aws/artefacts/nav2:humble-fortress

WORKDIR /ws

# -----------------------------
# Install Xvfb and necessary X11 libraries for headless Gazebo
# -----------------------------
RUN apt-get update && apt-get install -y \
    xvfb \
    x11-apps \
    xserver-xorg-core \
    xserver-xorg-video-dummy \
    xserver-xorg-video-all \
    libxrender1 \
    libx11-6 \
    libxtst6 \
    libxext6 \
    libosmesa6 \
    mesa-utils \
    python3-vcstool \
 && rm -rf /var/lib/apt/lists/*

# -----------------------------
# Import external dependencies
# -----------------------------
COPY deps.repos /tmp
RUN mkdir src && vcs import --input /tmp/deps.repos src

# Install ROS2 dependencies
RUN apt update -y && rosdep install --from-paths src --ignore-src -r -y

# First build to speed up rebuilds
RUN source /opt/ros/humble/setup.bash --extend && colcon build --symlink-install

# -----------------------------
# Copy your workspace
# -----------------------------
COPY . /ws

# Install extra ROS2 packages and dependencies
RUN apt update -y && apt install -y ros-humble-rosbag2-storage-mcap && \
    rosdep install --from-paths src --ignore-src -r -y

# Full build after copying workspace
RUN source /opt/ros/humble/setup.bash --extend && colcon build --symlink-install

# Install Python dependencies for your packages
RUN pip install -r src/sam_bot_nav2_gz/requirements.txt

# -----------------------------
# CMD: start Xvfb headless, set DISPLAY, launch ROS2
# -----------------------------
CMD ["bash", "-c", "\
    rm -f /tmp/.X99-lock && \
    pkill Xvfb || true && \
    Xvfb :99 -screen 0 1024x768x24 -nolisten tcp -noreset & \
    export DISPLAY=:99 && \
    source /opt/ros/humble/setup.bash && \
    source /ws/install/setup.bash && \
    ros2 launch sam_bot_nav2_gz complete_navigation.launch.py \
"]
