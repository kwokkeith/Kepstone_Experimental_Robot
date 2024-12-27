#!/bin/bash
# This script starts a Node.js server to serve web files to the sql database using ROS

# Navigate to the web directory
cd "$(rospack find bumperbot_graphical_interface)/web/ros-frontend/src"

node sqlserver.js

# exit 0