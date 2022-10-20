cmake_minimum_required(VERSION 3.8)
project(simple_slam)
set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(
  slam
  src/main.cc
  src/mainstream.cc
)

ament_target_dependencies(
  slam
  ament_cmake sensor_msgs std_msgs
  rclcpp Ceres PCL Eigen3 pcl_conversions
)

target_link_libraries(
  slam
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
)

install(
  TARGETS slam
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()