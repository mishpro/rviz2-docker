FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-jazzy-pinocchio \
    ros-jazzy-ament-cmake \
    ros-jazzy-launch \
    ros-jazzy-launch-ros \
    ros-jazzy-ament-index-python \
    build-essential cmake git wget \
    libeigen3-dev \
    mesa-utils \
    micro \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
RUN echo 'source /workspace/ros2_ws/install/setup.bash' >> ~/.bashrc

WORKDIR /workspace
RUN git clone https://github.com/TheRobotStudio/SO-ARM100.git
RUN mkdir -p ros2_ws/src
COPY app ros2_ws/src/so101_ik
COPY scripts /workspace/ros2_ws/src/so101_ik/scripts
WORKDIR /workspace/ros2_ws
RUN . /opt/ros/jazzy/setup.sh && colcon build --merge-install

ENV AMENT_PREFIX_PATH=/workspace/ros2_ws/install:/opt/ros/jazzy

CMD ["/bin/bash"]
