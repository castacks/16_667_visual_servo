cmake_minimum_required(VERSION 2.8.3)
project(core_drone_interface)

add_compile_options(-Wno-delete-non-virtual-dtor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pluginlib
  message_generation
  mav_msgs
)

find_package(base)


add_service_files(
  FILES
  DroneCommand.srv
  )

generate_messages(DEPENDENCIES std_msgs)

catkin_package()


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rospy roscpp std_msgs mav_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${base_INCLUDE_DIRS}
  include
)

add_executable(drone_interface_node src/drone_interface_node.cpp)

add_dependencies(drone_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${base_EXPORTED_TARGETS})

target_link_libraries(drone_interface_node
  ${catkin_LIBRARIES}
  ${base_LIBRARIES}
)

install(TARGETS drone_interface_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install (DIRECTORY launch srv
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})