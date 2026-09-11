cmake_minimum_required(VERSION 3.16)
project(so101_ik)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "/opt/ros/jazzy")
set(CMAKE_LIBRARY_ARCHITECTURE "x86_64-linux-gnu")

find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(ik_example so101-ik-node.cpp)
target_link_libraries(ik_example
    pinocchio::pinocchio
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
    visualization_msgs::visualization_msgs__rosidl_typesupport_cpp
)

# Установка launch и конфига
install(TARGETS ik_example DESTINATION lib/so101_ik)
install(DIRECTORY launch DESTINATION share/so101_ik)
install(DIRECTORY config DESTINATION share/so101_ik)

find_package(ament_cmake REQUIRED)
ament_package()
