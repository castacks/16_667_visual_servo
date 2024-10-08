cmake_minimum_required(VERSION 2.8.3)
project(core_pose_controller)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  core_pid_controller
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  mav_msgs
)

find_package(base)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pose_controller_lib
  CATKIN_DEPENDS roscpp tf rospy mav_msgs
)

include_directories(${catkin_INCLUDE_DIRS} include ${base_INCLUDE_DIRS})

add_library(pose_controller_lib
  src/pose_controller.cpp
)
add_dependencies(pose_controller_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_controller_lib ${catkin_LIBRARIES})

add_executable(pose_controller
  src/pose_controller_node.cpp
  src/pose_controller.cpp
)
add_dependencies(pose_controller ${catkin_EXPORTED_TARGETS} ${base_EXPORTED_TARGETS})
target_link_libraries(pose_controller ${catkin_LIBRARIES} ${base_LIBRARIES})

install(TARGETS pose_controller_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
