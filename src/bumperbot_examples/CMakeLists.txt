cmake_minimum_required(VERSION 3.0.2)
project(bumperbot_examples)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  turtlesim
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  turtlesim
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

add_library(turtlesim_kinematics src/turtlesim_kinematics.cpp)
add_dependencies(turtlesim_kinematics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtlesim_kinematics ${catkin_LIBRARIES})

add_executable(simple_cpp_publisher nodes/simple_publisher.cpp)
target_link_libraries(simple_cpp_publisher ${catkin_LIBRARIES})

add_executable(simple_cpp_subscriber nodes/simple_subscriber.cpp)
target_link_libraries(simple_cpp_subscriber ${catkin_LIBRARIES})

# Below executable ...node.cpp also requires the turtlesim_kinematics.cpp to work correctly
# hence, we compile the executable with the two cpp files.
add_executable(turtlesim_kinematics_node nodes/turtlesim_kinematics_node.cpp src/turtlesim_kinematics.cpp)
add_dependencies(turtlesim_kinematics_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# turtlesim_kinematics_node also needs the turtlesim_kinematics lib
target_link_libraries(turtlesim_kinematics_node turtlesim_kinematics ${catkin_LIBRARIES}) 