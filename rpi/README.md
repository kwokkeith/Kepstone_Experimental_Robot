# Cross-compiling ROS Binaries for Raspberry Pi 2 Zero W

A brief description of what your project does.

---

## Getting Started

Follow these steps to set up your development environment and run the project.

### System Requirements and Prerequisites

- x86-64 system
- Ubuntu 20.04 (Focal Fossa)
- Git

### Installation Steps

1. **Install ROS Noetic**
    ```bash
    sudo apt update && sudo apt install curl gnupg -y
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update && sudo apt install ros-noetic-ros-base -y
    
2. **Install Rust**
    ```bash
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

3. **Install aarch64 toolchain for Linux**
    ```bash
    rustup target add aarch64-unknown-linux-gnu 

4. **Download and extract Zig 0.14.0 compiler for x64**
    ```bash
    curl -s -L https://ziglang.org/download/0.14.0/zig-linux-x86_64-0.14.0.tar.xz | tar xvJ -C ~

5. **Automatically add zig binary to PATH environment variable**
    ```bash
    echo "export PATH=$PATH:$HOME/zig-linux-x86_64-0.14.0/" >> $HOME/.bashrc && source $HOME/.bashrc

6. **Test Zig compiler path works and correctly displays its version**
    ```bash
    zig version

7. **Install cargo-zigbuild to allow Rust to use zig as a linker when cross-compiling**
    ```bash
    cargo install --locked cargo-zigbuild 

8. **Clone this repository and change directory into the rosbin project directory**
    ```bash
    git clone https://github.com/agx-hv/dv8 && cd dv8/rpi/rosbin

9. **Build the arm64 binary**
    ```bash
    cargo zigbuild --release --target aarch64-unknown-linux-gnu




