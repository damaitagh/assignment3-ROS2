cmake_minimum_required(VERSION 3.8)
project(hikrobot_camera)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# Hikrobot MVS SDK
find_path(
  MVS_INCLUDE_DIR
  NAMES MvCameraControl.h
  PATHS
    /opt/MVS/include
    /opt/MVS/include/64
  REQUIRED
)

find_library(
  MVS_LIBRARY
  NAMES MvCameraControl
  PATHS
    /opt/MVS/lib/64
    /opt/MVS/lib
  REQUIRED
)

message(STATUS "MVS include: ${MVS_INCLUDE_DIR}")
message(STATUS "MVS library: ${MVS_LIBRARY}")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

add_executable(
  hikrobot_camera_node
  src/main.cpp
  src/camera_node.cpp
)

target_compile_features(
  hikrobot_camera_node
  PUBLIC
  cxx_std_17
)

target_include_directories(
  hikrobot_camera_node
  PRIVATE
    include
    ${MVS_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(
  hikrobot_camera_node
  rclcpp
  sensor_msgs
  rcl_interfaces
  std_msgs
  cv_bridge
)

target_link_libraries(
  hikrobot_camera_node
  ${MVS_LIBRARY}
  ${OpenCV_LIBS}
  Threads::Threads
)

set_target_properties(
  hikrobot_camera_node
  PROPERTIES
    BUILD_RPATH "/opt/MVS/lib/64;/opt/MVS/bin"
    INSTALL_RPATH "/opt/MVS/lib/64;/opt/MVS/bin"
)

install(
  TARGETS
    hikrobot_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    include/
  DESTINATION include
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
