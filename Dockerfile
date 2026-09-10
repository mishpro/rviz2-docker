FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-jazzy-pinocchio \
    build-essential cmake git wget \
    libeigen3-dev \
    mesa-utils \
    micro \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc

WORKDIR /workspace
RUN git clone https://github.com/TheRobotStudio/SO-ARM100.git
RUN mkdir app
COPY so101-ik.cpp app
COPY CMakeLists.txt app
RUN cd app && cmake -S . -B build && cmake --build build

CMD ["/bin/bash"]
