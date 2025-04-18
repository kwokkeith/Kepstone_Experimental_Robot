cmake_minimum_required(VERSION 3.0.2)
project(litter_destruction)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib
  geometry_msgs
  nav_msgs
  navfn
  message_generation
  bumperbot_detection
  visualization_msgs
  bumperbot_utils
  std_srvs
  move_base_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services, and actions ##
################################################

add_service_files(
  FILES
  GlobalBoundaryCenter.srv
  LocalBoundaryCenter.srv
  GetLitterSet.srv
  GetNextLitter.srv
  RemoveLitter.srv
  HasLitterToClear.srv
  GetNextTargetLitter.srv
)

# Uncomment if you have actions defined
# add_action_files(
#   FILES
#   Navigate.action
# )

# Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  actionlib_msgs
  bumperbot_detection
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    actionlib 
    actionlib_msgs
    geometry_msgs 
    bumperbot_detection
    nav_msgs
    navfn
    visualization_msgs
    message_runtime
    bumperbot_utils
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${bumperbot_utils_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Mark executable Python scripts for installation
catkin_install_python(PROGRAMS
  src/litter_destroy.py
  src/litter_destruction/litter_manager.py
  src/litter_manager_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Library for pixel position publisher
add_library(boundary_visualizer src/boundary_visualizer.cpp)
add_dependencies(boundary_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(boundary_visualizer ${catkin_LIBRARIES})

# Executable node for boundary visualizer
add_executable(boundary_visualizer_node nodes/boundary_visualizer_node.cpp)
target_link_libraries(boundary_visualizer_node ${catkin_LIBRARIES} boundary_visualizer)
