cmake_minimum_required(VERSION 3.0.2)
project(bumperbot_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
  visualization_msgs
  cv_bridge
  image_transport
)

find_package(Eigen3 REQUIRED)

# Add message files
add_message_files(
  FILES
  LitterList.msg
  LitterPoint.msg
  DetectedLitterPoint.msg
)

add_service_files(
  FILES
  AddLitter.srv
  DeleteLitter.srv
  GetLitterList.srv
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate added messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS 
   roscpp 
   rospy
   std_msgs
   geometry_msgs
   tf2
   tf2_ros
   tf2_geometry_msgs
   message_runtime
   visualization_msgs
   
   DEPENDS Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# # Link the libraries
# target_link_libraries(
#   ${catkin_LIBRARIES}
#   ${EIGEN3_LIBRARIES} 
# )

# Litter Coordinate Transformer Publisher library (Camera frame to base frame)
add_library(litter_coordinate_transformer_publisher src/litter_coordinate_transformer.cpp)
add_dependencies(litter_coordinate_transformer_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(litter_coordinate_transformer_publisher ${catkin_LIBRARIES})

# Litter Coordinate Transformer Publisher NODE 
add_executable(litter_coordinate_transformer_publisher_node nodes/litter_coordinate_transformer_publisher.cpp)
target_link_libraries(litter_coordinate_transformer_publisher_node litter_coordinate_transformer_publisher ${catkin_LIBRARIES})

# Litter memory node
add_executable(litter_memory_node nodes/litter_memory_node.cpp)
add_dependencies(litter_memory_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(litter_memory_node ${catkin_LIBRARIES})

# Litter Coordinate subscriber 
add_executable(litter_coordinate_subscriber nodes/litter_coordinate_subscriber.cpp)
target_link_libraries(litter_coordinate_subscriber ${catkin_LIBRARIES})

# Litter Marker Publisher
add_executable(litter_plotter_publisher_node nodes/litter_plotter.cpp)
target_link_libraries(litter_plotter_publisher_node ${catkin_LIBRARIES})
add_dependencies(litter_plotter_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Image Compressor
add_executable(image_compressor_node nodes/image_compressor.cpp)
target_link_libraries(image_compressor_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(image_compressor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})