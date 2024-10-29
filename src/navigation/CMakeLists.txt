cmake_minimum_required(VERSION 3.0.2)
project(navigation)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  move_base_msgs
  actionlib
  geometry_msgs
  message_generation
  bumperbot_utils
  litter_destruction
  tf
  nav_msgs
)

# Find the yaml-cpp library
find_package(yaml-cpp REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

# Declare services
add_service_files(
  FILES
  GetAmclPose.srv
  GetPixelPose.srv
  ConvertPixelToMap.srv
  ConvertPixelWaypointsToMap.srv
  InitiateCoveragePath.srv
  getWaypoints.srv
)

# Generate messages and services with any dependencies
generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs 
)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    std_msgs 
    geometry_msgs 
    message_runtime 
    bumperbot_utils 
    litter_destruction
    tf
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${bumperbot_utils_INCLUDE_DIRS}
)

# Library for pixel position publisher
add_library(pixel_position_publisher src/pixel_position_publisher.cpp)
add_dependencies(pixel_position_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pixel_position_publisher ${catkin_LIBRARIES} yaml-cpp)

# Library for waypoint manager
add_library(waypoint_manager src/waypoint_manager.cpp)
add_dependencies(waypoint_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_manager ${catkin_LIBRARIES})

# Executable for waypoint manager
add_executable(waypoint_manager_node nodes/waypoint_manager_node.cpp)
add_dependencies(waypoint_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(waypoint_manager_node ${catkin_LIBRARIES} waypoint_manager utils)

# Executable node for pixel position publisher
add_executable(pixel_position_publisher_node nodes/pixel_position_publisher_node.cpp)
add_dependencies(pixel_position_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(pixel_position_publisher_node ${catkin_LIBRARIES} pixel_position_publisher utils)

# Executable node for get_amcl_pose_server
add_executable(get_amcl_pose_server_node src/get_amcl_pose_server.cpp)
add_dependencies(get_amcl_pose_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(get_amcl_pose_server_node ${catkin_LIBRARIES})

# Executable node for get_pixel_pose_server
add_executable(get_pixel_pose_server_node src/get_pixel_pose_server.cpp)
add_dependencies(get_pixel_pose_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(get_pixel_pose_server_node ${catkin_LIBRARIES} utils)

# Executable node for convert_pixel_to_map_server
add_executable(convert_pixel_to_map_server_node src/convert_pixel_to_map_server.cpp)
add_dependencies(convert_pixel_to_map_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(convert_pixel_to_map_server_node ${catkin_LIBRARIES} utils)

# Executable node for convert_pixels_to_mapwaypoints_server
add_executable(convert_pixel_waypoints_to_map_waypoints_server_node src/convert_pixels_to_mapwaypoints_server.cpp)
add_dependencies(convert_pixel_waypoints_to_map_waypoints_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(convert_pixel_waypoints_to_map_waypoints_server_node ${catkin_LIBRARIES} utils)

# Executable waypoint plotter
add_executable(waypoint_plotter_publisher_node src/waypoint_plotter.cpp)
add_dependencies(waypoint_plotter_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_plotter_publisher_node ${catkin_LIBRARIES})

#############
## Install ##
#############

# Install Python programs
catkin_install_python(PROGRAMS
  src/waypoint_nav.py
  nodes/robot_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
