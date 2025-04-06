# DV8 Cleaning Robot ROS Navstack Project

---

## bumperbot_controller

Contains nodes and launch files to manage robot control states and movements.

### Nodes:
- **robot_controller_node.py**: Controls robot state machine (IDLE, COVERAGE, TRANSITION, LITTER_PICKING).
- **litter_tracker** *(Deprecated)*: Handles LITTER_TRACKING mode.
- **move_manager**: Manages robot movement behaviours and interfaces with the navstack.
- **noisy_controller** *(Simulation)*: Simulates odometry noise.
- **simple_controller** *(Simulation)*: Processes joint and velocity commands for move_base.
- **vacuum_controller**: Controls vacuum hardware.
- **rollerbrush_controller**: Controls rollerbrush hardware.
- **sidebrush_controller**: Controls sidebrush hardware.

### Launch Files:
- `controller.launch` *(Simulation)*: Launches ROS move_base node.
- `joystick_teleop.launch`: Enables joystick teleoperation.
- `keyboard_teleop.launch`: Enables keyboard teleoperation.
- `robot_controller.launch`: Launches essential robot controller nodes.

---

## bumperbot_description

Holds robot URDF files and Gazebo world launch files.

### Launch Files:
- `display.launch`: Visualises robot and TF transforms in RViz.
- `gazebo_test.launch`: Loads a test world in Gazebo with the robot.
- `gazebo.launch`: Loads an empty Gazebo world with the robot.

---

## bumperbot_detection

Contains nodes and codes for computer vision-based object detection and TF transforms.

### Nodes:
- **cv_api_get_coordinates**: Detects objects and publishes coordinates.
- **get_coordinates**: Transforms camera frame coordinates (front camera).
- **get_coordinates_rear**: Transforms camera frame coordinates (rear camera).
- **litter_coordinate_transformer**: Converts detected coordinates to map frame.
- **yolov8_get_coordinates** *(Simulation)*: Uses YOLOv8 model for simulation.
- **litter_memory_node**: Maintains litter detection memory.
- **litter_plotter**: Visualises litter memory in RViz.
- **image_compressor**: Compresses video feeds for publishing.

### Launch Files:
- `litter_detection.launch`: Launches detection nodes for simulation.

---

## bumperbot_graphical_interface

Contains the React Web UI for robot management and database.

### Launch Files:
- `start_webapp.launch`: Starts the robot web UI.

---

## bumperbot_localization

Manages robot localisation (simulation).

### Nodes:
- **kalman_filter**: Kalman filter for localisation from odometry, IMU, laser scans.
- **lidar_fusion**: Fuses LiDAR scans into a unified point cloud.
- **imu_republisher_node**: Republishes IMU data after Extended Kalman Filter processing.

### Launch Files:
- `local_localization.launch`: Starts localisation nodes.

---

## bumperbot_utils

Utility nodes for robot navigation stack operations.

### Nodes:
- **path_plotter_node**: Visualises robot path.
- **utils**: Utilities for map loading, waypoint handling, coordinate conversions.

---

## config_manager

Contains global configuration parameters for the robot navstack.

---

## coverage_planner

Handles coverage path planning using Boustrophedon Cellular Decomposition.

### Launch Files:
- `coverage_planner.launch`: Starts coverage planner node for web interface.

---

## i2r_adapter *(Deprecated)*

Interface for i2r navstack MRCCC (not maintained).

---

## litter_destruction

Nodes managing litter removal tasks in LITTER_PICKING mode.

### Nodes:
- **litter_destroy** *(Deprecated)*: Litter destruction algorithm relying on `litter_manager.py`.
- **litter_manager**: Manages litter memory, boundary definitions, and litter clearance processes.
- **litter_manager_node**: Runs the litter manager node.
- **boundary_visualizer**: Visualises litter boundaries in RViz.

### Launch Files:
- `litter_manager.launch`: Starts litter manager and boundary visualiser nodes.

---

## navigation

Handles waypoint creation for coverage path planning and interfaces with `move_base`.

### Nodes:
- **convert_pixel_to_map_server**: Converts single pixel points to map coordinates.
- **convert_pixels_to_mapwaypoints_server**: Converts multiple pixel points to map coordinates.
- **get_amcl_pose_server**: Provides robot's AMCL position.
- **get_pixel_pose_server**: Provides robot's pixel position.
- **pixel_position_publisher**: Publishes robot’s current pixel position.
- **waypoint_manager**: Manages robot coverage waypoints and navigation.
- **waypoint_nav** *(Deprecated)*: Issues movement commands for coverage paths.
- **waypoint_plotter**: Visualises coverage paths in RViz.

---

## websocket_comm *(Deprecated)*

Previously used for communication with i2r MRCCC.

