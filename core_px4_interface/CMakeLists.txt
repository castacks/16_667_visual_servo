cmake_minimum_required(VERSION 2.8.3)
project(core_px4_interface)

add_compile_options(-Wno-unused-parameter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  core_drone_interface
  pluginlib
  mavros_msgs
  message_generation
)

find_package(base)

add_message_files(
  FILES
  AttMode.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs 
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${base_INCLUDE_DIRS}
  include
)

add_library(px4_interface src/px4_interface.cpp)
add_library(px4_no_tilt_interface src/px4_no_tilt_interface.cpp)
add_library(px4_tilted_hex_interface src/px4_tilted_hex_interface.cpp)
add_executable(px4_offboard_example_node src/px4_offboard_example_node.cpp)

add_dependencies(px4_offboard_example_node ${catkin_EXPORTED_TARGETS})
add_dependencies(px4_tilted_hex_interface core_px4_interface_generate_messages_cpp)

target_link_libraries(px4_offboard_example_node
  ${catkin_LIBRARIES}
#  ${base_LIBRARIES}
)

install(TARGETS px4_interface px4_no_tilt_interface px4_tilted_hex_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install (DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install (DIRECTORY .
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "px4*.xml")
