cmake_minimum_required(VERSION 2.8.3)
project(gazebo_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  core_drone_interface
  pluginlib
)

find_package(base)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${base_INCLUDE_DIRS}
  include
)

add_library(gazebo_interface src/gazebo_interface.cpp)

#add_executable(gazebo_interface_node src/gazebo_interface_node.cpp)

#add_dependencies(gazebo_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${base_EXPORTED_TARGETS})

#target_link_libraries(gazebo_interface_node
#  ${catkin_LIBRARIES}
#  ${base_LIBRARIES}
#)
