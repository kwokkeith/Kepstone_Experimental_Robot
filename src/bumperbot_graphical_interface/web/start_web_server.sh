#!/bin/bash
# This script starts a Python HTTP server to serve web files from the ROS package

# Navigate to the web directory
cd "$(rospack find bumperbot_graphical_interface)/web/frontend"

# Start the Python HTTP server
python3 -m http.server 8000
