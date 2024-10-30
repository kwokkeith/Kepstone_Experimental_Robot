cmake_minimum_required(VERSION 3.0.2)
project(bumperbot_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  message_generation
  tf2
  tf2_ros
  navigation
  litter_destruction
)

find_package(Eigen3 REQUIRED)

# Declare services
add_service_files(
  FILES
  ModeSwitch.srv
)

# Declare messages
generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs 
    navigation
    litter_destruction
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    message_runtime
    tf2
    tf2_ros
    navigation
    litter_destruction
  DEPENDS Eigen3
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


#############
## Install ##
#############

# Install Python programs
catkin_install_python(PROGRAMS
  src/bumperbot_controller/nodes/robot_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(simple_controller src/simple_controller.cpp)
add_dependencies(simple_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_controller ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_library(noisy_controller src/noisy_controller.cpp)
add_dependencies(noisy_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(noisy_controller ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(simple_controller_node nodes/simple_controller_node.cpp)
add_dependencies(simple_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_controller_node ${catkin_LIBRARIES} simple_controller)

add_executable(noisy_controller_node nodes/noisy_controller_node.cpp)
add_dependencies(noisy_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(noisy_controller_node ${catkin_LIBRARIES} noisy_controller)


#############
## Testing ##
#############