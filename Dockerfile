# Hybrid A* planner with trajectory post-processing + Ceres back-end optimization.
# ROS Noetic, builds the catkin workspace and launches with rviz (needs X11).
FROM osrf/ros:noetic-desktop-full

# Avoid interactive tzdata prompts during apt installs.
ENV DEBIAN_FRONTEND=noninteractive

# --- System + ROS dependencies ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        libeigen3-dev \
        libgoogle-glog-dev \
        libceres-dev \
        ros-noetic-costmap-2d \
        ros-noetic-map-server \
        ros-noetic-tf \
        ros-noetic-joint-state-publisher \
        ros-noetic-joint-state-publisher-gui \
        ros-noetic-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# --- Build the catkin workspace ---
ENV WS=/root/hybrid_a_star_ws
RUN mkdir -p ${WS}/src
COPY . ${WS}/src/hybrid_a_star

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash \
    && cd ${WS} \
    && catkin_make -DCMAKE_BUILD_TYPE=Release"

# --- Entry point sources ROS + the workspace, then runs the given command ---
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ["roslaunch", "hybrid_a_star", "run_hybrid_a_star.launch"]
