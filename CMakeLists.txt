cmake_minimum_required(VERSION 3.14)
project(ovinf_ros2)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

set(BITBOT_DEPENDENCY_USE_PROXY TRUE)
set(BITBOT_DEPENDENCY_USE_LOCAL_FILE TRUE)
set(BITBOT_DEPENDENCY_LOCAL_FILE_PATH ${CMAKE_SOURCE_DIR}/dependencies)

include(dependencies/bitbot.cmake)
include_directories("/usr/include/eigen3")

add_definitions(-DBITBOT_DEPLOY)
set(CIFX_HEADER "/home/dknt/Library/cifx/include/")
include_directories("/home/dknt/Library/cifx/include/")
link_directories("/home/dknt/Library/cifx/lib/")
BitbotAddDependency(
  NAME           bitbot_cifx
  FILE_NAME      "bitbot_cifx-main.zip"
  FILE_PATH      ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
  USE_LOCAL_FILE TRUE
)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBNL REQUIRED libnl-3.0)
pkg_check_modules(LIBNL_ROUTE REQUIRED libnl-route-3.0)

include(FetchContent)
FetchContent_Declare(
  ovinf
  GIT_REPOSITORY https://github.com/Dknt0/ovinf.git
  GIT_TAG main
)
FetchContent_MakeAvailable(ovinf)

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Ovinf dependencies
find_package(OpenVINO REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(main_app main.cc)
target_include_directories(main_app PUBLIC
  ${OVINF_INCLUDED}
  ${OpenVINO_INCLUDE_DIRS}
  "/usr/include/eigen3/"
  ${CMAKE_SOURCE_DIR}/include
)
ament_target_dependencies(main_app
  rclcpp
  sensor_msgs
  grid_map_ros
  grid_map_msgs
  tf2_ros
)
target_link_libraries(main_app
  BitbotCifx
  BitbotKernel
  ovinf
  # ${LIBNL_LIBRARIES}
  # "/lib/x86_64-linux-gnu/libnl-cli-3.so"
  # ${LIBNL_ROUTE_LIBRARIES}
)

target_compile_options(main_app PUBLIC)
target_compile_features(main_app PUBLIC cxx_std_20)

add_executable(joint_state_test test/joint_state_test.cc)
ament_target_dependencies(joint_state_test
  rclcpp
  sensor_msgs
  ament_index_cpp
)
target_link_libraries(joint_state_test
  yaml-cpp
)

install(TARGETS
  main_app joint_state_test
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch config rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
