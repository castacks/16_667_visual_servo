cmake_minimum_required(VERSION 2.8.3)
project(core_behavior_executive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  behavior_tree
  behavior_tree_msgs
  core_takeoff_landing_planner
  core_trajectory_controller
  core_drone_interface
  core_trajectory_msgs
)

find_package(base)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${base_INCLUDE_DIRS}
  include
)

add_executable(behavior_executive_behavior_tree src/behavior_executive_behavior_tree.cpp)

add_dependencies(behavior_executive_behavior_tree ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${base_EXPORTED_TARGETS})

target_link_libraries(behavior_executive_behavior_tree
  ${catkin_LIBRARIES}
  ${base_LIBRARIES}
)
