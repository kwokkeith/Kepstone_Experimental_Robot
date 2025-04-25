# Source Files
- `rosbin/src/main.rs`    
    - Entrypoint, used to select which code to compile
- `rosbin/src/pwm_init.rs`
    - Initialize GPIO and PWM to the appropriate modes, must be run before sidebrush/vacuum binaries
- `rosbin/src/sidebrush.rs`
    - Sidebrush source code
- `rosbin/src/vacuum.rs`
    - Vacuum and rollerbrush source code
- `rosbin/src/servo.rs`
    - Contains struct to interface with SG09 servo via PWM, used by vacuum.rs
- `rosbin/src/bts7960_motor_control.rs`
    - Contains struct to interface with BTS7960 Motor driver via PWM, used by sidebrush.rs

# ROS Messages
- `rosbin/rosrust_msgs/bumperbot_controller/msg/RollerBrushPosFeedback.msg` (unused) 
- `rosbin/rosrust_msgs/bumperbot_controller/msg/RollerBrushPowerFeedback.msg`
    - power: 0.0 to 1.0
    - message: "OK" or "ERROR"
    - pos: "UP" or "DOWN"
- `rosbin/rosrust_msgs/bumperbot_controller/msg/SideBrushPosFeedback.msg` (unused) 
- `rosbin/rosrust_msgs/bumperbot_controller/msg/SideBrushSpeedFeedback.msg`
    - speed: 0 to 300
    - message: "OK" or "ERROR"
    - pos: "UP" or "DOWN"
- `rosbin/rosrust_msgs/bumperbot_controller/msg/VacuumPowerFeedback.msg` (unused) 

# Cross-Compiling ROS Noetic Rust Binaries for Raspberry Pi Zero 2 W

This guide walks you through the process of cross-compiling ROS Noetic binaries for the Raspberry Pi Zero 2 W, which uses an ARM64 processor. Rather than compiling directly on the Pi — which is slow and resource-limited — this approach allows you to build binaries on a fast x86-64 machine and run them immediately on a fresh install of Raspberry Pi OS.

⚠️ Note: ROS Noetic is only officially supported on Ubuntu 20.04 (Focal), which reaches end-of-Life (EOL) in May 2025. It is strongly recommended to use Ubuntu 20.04 in a chroot, VM, or container for building, rather than on your host system.

---
### ⚙️ Why Cross-Compile for the Pi?

Cross-compiling provides more than just faster build times. Here are some major advantages:

✅ Avoid Using Outdated OSes on the Pi

ROS Noetic is officially supported only on Ubuntu 20.04 Focal, which reached End-of-Life in May 2025. Installing and running Focal on a Raspberry Pi is not only clunky and unsupported, but also introduces long-term security risks and compatibility issues.

By cross-compiling, you don’t need to install Ubuntu on the Pi at all — you can run your binaries directly on the latest, officially supported Raspberry Pi OS.

This means:
- Your Pi stays up to date with Debian-based system libraries.
- You benefit from ongoing security updates and hardware compatibility.
- There's no need to hack around outdated Ubuntu images just to run ROS.

✅ No Need to Install ROS on the Pi

You don’t even need to install ROS on the Pi at all. The compiled binaries can be copied over and executed immediately, greatly reducing the system footprint and avoiding complex dependency management.

This simplifies deployment and makes it easy to:
- Run ROS nodes on multiple devices without repeated setup.
- Maintain a clean, lightweight Pi image.
- Isolate and reproduce issues, since everything is built in a controlled dev environment.

✅ Faster and More Scalable Development
- You compile on a powerful x86-64 machine, not the slow ARM CPU on the Pi.
- Builds that take 30+ minutes on the Pi complete in seconds on your dev machine.
- Ideal for CI/CD pipelines or large ROS workspaces.

✅ Reproducible and Consistent Binaries
- The build process is deterministic — every Pi gets the same binary.
- Easy to debug and version control.
- Compatible with system libraries on Raspberry Pi OS out of the box.

### 🔗 Why Use Zig to Link Rust?
Traditional cross-compilation often requires setting up:

- A cross toolchain (e.g., gcc-aarch64-linux-gnu)
- A compatible sysroot and linker
- Manual configuration of compiler flags and environment variables

Zig replaces all of that with a single, portable, zero-setup linker.

🔧 Zig Makes Cross-Compilation Simple:
- Automatically configures the correct linker and system libraries for the target.
- No need to mess with LD_LIBRARY_PATH, sysroots, or architecture-specific GCC setups.
- Works seamlessly with Rust via cargo-zigbuild.

✅ Cross-Compiles Rust to ARM64 Without Hassle:
- Just run cargo zigbuild --target aarch64-unknown-linux-gnu and you're done.
- No need to install or configure complex toolchains.
- The resulting binary will link correctly against the Pi’s system libraries out-of-the-box.

### 🦀 Why Write ROS Nodes in Rust?
Rust is a modern systems programming language designed for performance, reliability, and safety — all of which make it an excellent fit for developing ROS nodes on embedded platforms like the Raspberry Pi Zero 2 W.

Here’s why Rust is a great choice for this cross-compilation workflow:

✅ Memory Safety Without Runtime Overhead

Rust guarantees memory safety at compile time, without relying on a garbage collector. This eliminates entire classes of bugs — such as null pointer dereferences, buffer overflows, and use-after-free — while still delivering performance close to C/C++. This is especially important on embedded devices where resources are limited, and bugs can be hard to debug.

✅ Lightweight, High-Performance Binaries

Rust compiles directly to efficient native code with zero-cost abstractions. The resulting binaries are small, fast, and ideal for running on the Pi Zero 2 W, which has limited CPU and memory resources.

✅ Safe Concurrency by Design

Rust’s ownership system also applies to concurrent programming. It makes data races impossible by default, allowing you to confidently build multithreaded ROS nodes that handle multiple topics, services, or sensors in parallel — without risking undefined behavior or hard-to-find bugs.

✅ Writing ROS Nodes in Rust

With libraries like rosrust, you can write fully-functional ROS Noetic nodes in pure Rust — supporting publishers, subscribers, services, and parameters. This brings all of Rust’s safety and performance benefits into the ROS ecosystem, without needing to fall back on C++.


### System Requirements and Prerequisites

- x86-64 Development Machine
- Ubuntu 20.04 LTS (in VM, Docker container, or chroot)
- Git

### 🏁 Quickstart (TL;DR) 

1. Clone this repo 
2. Build with Zig linker for ARM64: 
    ```bash 
    cargo zigbuild --release --target aarch64-unknown-linux-gnu 
3. SCP the binary to your Raspberry Pi and run!

### Detailed Installation Steps

1. **Install ROS Noetic on Ubuntu 20.04 LTS**
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

4. **Download and extract Zig 0.9.0 compiler for x64**
    ```bash
    curl -s -L https://ziglang.org/download/0.9.0/zig-linux-x86_64-0.9.0.tar.xz | tar xvJ -C ~

5. **Automatically add zig binary to PATH environment variable**
    ```bash
    echo "export PATH=$PATH:$HOME/zig-linux-x86_64-0.9.0/" >> $HOME/.bashrc && source $HOME/.bashrc

6. **Test Zig compiler path works and correctly displays its version**
    ```bash
    zig version

7. **Install cargo-zigbuild to allow Rust to use zig as a linker when cross-compiling**
    ```bash
    cargo install --locked cargo-zigbuild 

8. **Clone this repository and change directory into the rosbin project directory**
    ```bash
    git clone https://github.com/agx-hv/dv8 && cd dv8/rpi/rosbin

9. **Source ros environment and set the ROSRUST_MSG_PATH environment variable**
    ```bash
    source /opt/ros/noetic/setup.bash && export ROSRUST_MSG_PATH=/path/to/directory/containing/bumperbot_controller

10. **Build the arm64 binary**
    ```bash
    cargo zigbuild --release --target aarch64-unknown-linux-gnu

11. **Copy the binary over to the Raspberry Pi**
    ```bash
    scp target/aarch64-unknown-linux-gnu/release/rosbin dv8@<RASPI_IP_ADDRESS>:~/


