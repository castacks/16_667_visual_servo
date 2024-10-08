cmake_minimum_required(VERSION 2.8.3)
project(core_takeoff_landing_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  behavior_tree
  core_trajectory_library
  core_trajectory_controller
  tflib
  core_trajectory_msgs
  message_generation
)

find_package(base)

add_service_files(
  FILES
  TakeoffLandingCommand.srv
  )

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${base_INCLUDE_DIRS}
  include
)

add_executable(takeoff_landing_planner src/takeoff_landing_planner.cpp)

add_dependencies(takeoff_landing_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${base_EXPORTED_TARGETS})

target_link_libraries(takeoff_landing_planner
  ${catkin_LIBRARIES}
  ${base_LIBRARIES}
)
