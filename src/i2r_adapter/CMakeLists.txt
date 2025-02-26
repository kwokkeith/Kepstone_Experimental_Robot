cmake_minimum_required(VERSION 3.0.2)
project(i2r_adapter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
find_package(Threads REQUIRED)

find_library(jsoncpp /lib/x86_64-linux-gnu)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SendJson.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES i2r-adapter
 CATKIN_DEPENDS message_runtime roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Define an executable target
add_executable(websocket_client_node nodes/client_node.cpp src/client.cpp src/mission_gen.cpp)

target_link_libraries(  
  websocket_client_node
  ${catkin_LIBRARIES}
  jsoncpp
  Threads::Threads
  crypto
  ssl
)

#############
## Install ##
#############


#############
## Testing ##
#############

