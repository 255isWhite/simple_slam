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

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIRS}/include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(
  slam
  src/main.cc
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