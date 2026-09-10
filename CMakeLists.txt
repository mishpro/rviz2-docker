cmake_minimum_required(VERSION 3.16)
project(so101_ik)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS 2 multiarch: конфиг Pinocchio лежит в lib/x86_64-linux-gnu/cmake/
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/jazzy")
set(CMAKE_LIBRARY_ARCHITECTURE "x86_64-linux-gnu")

find_package(pinocchio REQUIRED)

add_executable(so101-ik so101-ik.cpp)
target_link_libraries(so101-ik pinocchio::pinocchio)
