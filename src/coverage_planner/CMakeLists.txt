cmake_minimum_required(VERSION 3.0.2)
project(coverage_planner)

# Add ROS dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  roslib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES coverage_planner
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge roslib
  DEPENDS Eigen3 OpenCV CGAL
)

# Add your other dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(CGAL QUIET COMPONENTS Core)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
  include  # Your include directory
)

# Collect source files
file(GLOB_RECURSE srcs "src/*.cc" "src/*.cpp")
file(GLOB_RECURSE hdrs "include/*.h")

# Define the ROS node
add_executable(coverage_planner_node ${srcs} ${hdrs})

# Link libraries
target_link_libraries(coverage_planner_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  CGAL::CGAL
  CGAL::CGAL_Core
  pthread
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)