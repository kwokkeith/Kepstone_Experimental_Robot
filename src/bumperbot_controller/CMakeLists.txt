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
  move_base
  actionlib
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
  GetCurrentMode.srv
  SetVacuumPower.srv
  GetVacuumPower.srv
  SetRollerBrushPower.srv
  GetRollerBrushPower.srv
  SetRollerBrushDirection.srv
  GetRollerBrushDirection.srv
  SetSideBrushSpeed.srv
  GetSideBrushSpeed.srv
  SetSideBrushDirection.srv
  GetSideBrushDirection.srv
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
    move_base
    actionlib
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

add_library(move_manager src/move_manager.cpp)
add_dependencies(move_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_manager ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(move_manager_node nodes/move_manager_node.cpp)
add_dependencies(move_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_manager_node ${catkin_LIBRARIES} move_manager)

add_library(litter_tracker src/litter_tracker.cpp)
add_dependencies(litter_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(litter_tracker ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(litter_tracker_node nodes/litter_tracker_node.cpp)
add_dependencies(litter_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(litter_tracker_node ${catkin_LIBRARIES} litter_tracker)

add_library(vacuum_controller src/vacuum_controller.cpp)
add_dependencies(vacuum_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vacuum_controller ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(vacuum_controller_node nodes/vacuum_controller_node.cpp)
add_dependencies(vacuum_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vacuum_controller_node ${catkin_LIBRARIES} vacuum_controller)

add_library(rollerbrush_controller src/rollerbrush_controller.cpp)
add_dependencies(rollerbrush_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rollerbrush_controller ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(rollerbrush_controller_node nodes/rollerbrush_controller_node.cpp)
add_dependencies(rollerbrush_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rollerbrush_controller_node ${catkin_LIBRARIES} rollerbrush_controller)

add_library(sidebrush_controller src/sidebrush_controller.cpp)
add_dependencies(sidebrush_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sidebrush_controller ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(sidebrush_controller_node nodes/sidebrush_controller_node.cpp)
add_dependencies(sidebrush_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sidebrush_controller_node ${catkin_LIBRARIES} sidebrush_controller)

#############
## Testing ##
#############