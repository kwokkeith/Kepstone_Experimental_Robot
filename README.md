
# DV8 Robot Repository

## Project Name
Simulation for DV8 Spot Cleaning and Coverage Planning Algorithm <br><br>
Note: This repository is intended solely for simulating the performance of the implemented algorithms and does not accurately reflect the behavior of the actual robot.

## Installation
Clone the repository and set up the workspace:
```sh
git clone https://github.com/kwokkeith/Kepstone_Experimental_Robot.git
rosdep install --from-paths src --ignore-src -r -y
catkin_make  
source devel/setup.bash  
```

## Additional dependencies
Copy to install additional dependencies
```
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard ros-noetic-amcl ros-noetic-map-server ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro ros-noetic-rqt-image-view ros-noetic-gmapping ros-noetic-navigation ros-noetic-robot-state-publisher ros-noetic-slam-gmapping ros-noetic-dwa-local-planner ros-noetic-joint-state-publisher-gui ros-noetic-map-server ros-noetic-velodyne ros-noetic-velodyne-description ros-noetic-velodyne-driver ros-noetic-velodyne-gazebo-plugins ros-noetic-velodyne-laserscan ros-noetic-velodyne-msgs ros-noetic-velodyne-pointcloud ros-noetic-pointcloud-to-laserscan ros-noetic-robot-localization libwebsocketpp-dev
```


## Usage
### Launching the simulation (Gazebo and RViz)
The following command launches robot and CV simulation tools, and all other relevant nodes:
```sh
roslaunch bumperbot_examples test_env.launch 
```

### Litter Picking simulation
A small sphere can be placed in Gazebo to simulate a litter. <br>
To simulate litter picking mode:
```sh
rosservice call /robot_controller/start_robot_job
```

### Coverage Planner simulation
To simulate coverage planner mode:
```sh
rosservice call /robot_controller/initiate_coverage "waypoints_file: '/path/to/waypoints.txt'"
```
A sample waypoints file is available at:
```
/source_dir/Kepstone_Experimental_Robot/src/navigation/waypoints/waypoints.txt
```


## Configuration
Topic and Service parameters can be found in:
```
config_manager/config/parameters.yaml
```
Robot related parameters can be found in:
```
config_manager/config/robot_config.yaml
```


## Troubleshooting
| Issue  | Solution |
|--------|----------|
| ROS_PACKAGE_PATH not found | Run `source devel/setup.bash` (ROS 1) |
| Missing dependencies | Run `rosdep install --from-paths src --ignore-src -r -y`. <br> If the issue persists, check the 'Additional dependencies' section and try the suggested command. |
| Unable to find node | Ensure the package is built and sourced correctly |


## Contributors
SUTD Capstone Team S04


