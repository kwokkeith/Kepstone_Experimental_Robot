cmake_minimum_required(VERSION 3.0.2)
project(bumperbot_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
  )

# Find the yaml-cpp library
find_package(yaml-cpp REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

#############
## Install ##
#############
# Library for path plotting
add_library(path_plotter src/path_plotter.cpp)
add_dependencies(path_plotter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_plotter ${catkin_LIBRARIES})

# Executable for path plotting
add_executable(path_plotter_node nodes/path_plotter_node.cpp)
add_dependencies(path_plotter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_plotter_node ${catkin_LIBRARIES} path_plotter)

# Utility library
add_library(utils src/utils.cpp)
add_dependencies(utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(utils ${catkin_LIBRARIES} yaml-cpp)
