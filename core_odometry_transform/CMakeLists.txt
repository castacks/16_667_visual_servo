cmake_minimum_required(VERSION 2.8.3)
project(core_odometry_transform)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  tflib
)

find_package(base)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${base_INCLUDE_DIRS}
  include
)

add_executable(odometry_transform src/odometry_transform.cpp)

add_dependencies(odometry_transform ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${base_EXPORTED_TARGETS})

target_link_libraries(odometry_transform
  ${catkin_LIBRARIES}
  ${base_LIBRARIES}
)
