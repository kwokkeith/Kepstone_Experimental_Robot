# Cross-compiling ROS Binaries for Raspberry Pi 2 Zero W

A brief description of what your project does.

---

## Getting Started

Follow these steps to set up your development environment and run the project.

### Prerequisites

Make sure you have the following installed:

- Ubuntu 20.04 (Focal Fossa)
- Git
- Any other dependencies you need

### Installation Steps

1. **Install ROS Noetic**
    ```bash
    sudo apt update && sudo apt install gnupg -y
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update && sudo apt install ros-noetic-ros-base -y
    
2. **Install Rust**
    ```bash
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

3. **Install aarch64 toolchain**
