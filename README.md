# tasc_science

ROS 2 workspace for the TASC science module. This repository contains the robot description, `ros2_control` hardware interface, controller configuration, keyboard teleop, and launch files used to control and visualize the science subsystem.

The science module currently includes:

- **Lift linear actuator** controlled through an ESP32 and BTS7960 motor driver
- **Auger stepper motor** controlled through a Phidgets stepper controller
- **Beaker tray servo** with discrete positions at 0°, 90°, and 180°
- **URDF model** for RViz visualization
- **ros2_control hardware interface** for the physical hardware
- **Keyboard teleop** for manual operation
- Separate **backend** and **frontend** launch files for Jetson and base-station operation

## Repository layout

```text
tasc_science/
└── src/
    ├── science_description/
    │   ├── urdf/
    │   │   └── science_2026.urdf.xacro
    │
    ├── science_control/
    │   ├── config/
    │   │   └── controllers.yaml
    │   ├── include/science_control/
    │   │   └── science_2026_system.hpp
    │   ├── src/
    │   │   └── science_2026_system.cpp
    │   ├── teleop/
    │   │   └── science_2026_keyboard_teleop.cpp
    │   └── plugin.xml
    │
    └── science_bringup/
        └── launch/
            ├── bringup.launch.py
            ├── sci_backend.launch.py
            └── sci_frontend.launch.py
```

## Packages

### `science_description`

Contains the science-module robot model used by RViz and `robot_state_publisher`.

The URDF defines three controlled joints:

- `science_lift_joint`
- `auger_spin_joint`
- `beaker_tray_joint`

### `science_control`

Contains:

- The `ros2_control` hardware interface plugin
- Controller configuration
- Keyboard teleop node
- ESP32 serial communication
- Phidgets stepper control

The hardware interface connects:

- The lift actuator and beaker servo through the ESP32
- The auger stepper through the Phidgets API

### `science_bringup`

Contains the launch files used to start the system.

#### Backend launch

Runs on the NVIDIA Jetson and starts:

- `ros2_control_node`
- Science hardware interface
- `joint_state_broadcaster`
- Science velocity controller
- Beaker position controller

Key file:

```text
src/science_bringup/launch/sci_backend.launch.py
```

#### Frontend launch

Runs on the base-station VM and starts:

- Keyboard teleop
- `robot_state_publisher`
- RViz

Key file:

```text
src/science_bringup/launch/sci_frontend.launch.py
```

#### Regular launch

Runs on the NVIDIA Jetson but hides simulation.

- Keyboard teleop must be started on base-station VM

Key file:

```text
src/science_bringup/launch/bringup.launch.py
```

## Controls

### Lift actuator

Hold:

```text
W: Extend
S: Retract
```

The lift stops shortly after the key is released.

### Auger

```text
Q: Start forward rotation
A: Stop forward rotation

A: Start reverse rotation
Q: Stop reverse rotation
```

The auger operates as a three-state system:

```text
Reverse ↔ Neutral ↔ Forward
```

### Beaker tray

```text
E: Cycle tray position
```

Tray sequence:

```text
0° → 90° → 180° → 0°
```

The transition from 180° back to 0° is a direct 180° movement.

### Emergency stop

```text
Space: Stop the lift actuator and auger
```

## Dependencies

This project targets Ubuntu 22.04 and ROS 2 Humble.

Install the common ROS dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-rviz2 \
  ros-humble-robot-state-publisher \
  ros-humble-controller-manager \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-broadcaster \
  ros-humble-forward-command-controller \
  ros-humble-xacro
```

The Jetson also requires the Phidgets22 library and development headers for the auger stepper controller.

The ESP32 must be programmed with the matching science firmware and connected to the Jetson over USB serial.

## Build

Build the workspace on both the Jetson and the base-station VM.

From the workspace root:

```bash
cd ~/tasc_science
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The Jetson and VM should use matching versions of:

- `science_description`
- `science_control`
- `science_bringup`

## Network configuration

The Jetson and VM must be connected to the same network and use compatible ROS 2 discovery settings.

On both systems:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

Both systems should also use the same ROS middleware implementation. For example:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

These values can be added to `~/.bashrc`:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Then reload the shell:

```bash
source ~/.bashrc
```

## Run

### 1. Start the backend on the Jetson

After setting up network switch, SSH into the Jetson from the base-station VM:

```bash
ssh tasc@192.168.1.7
```

Then source the workspace and launch the backend:

```bash
cd ~/tasc_science
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch science_bringup sci_backend.launch.py
```

The backend connects to the physical hardware and starts all `ros2_control` controllers.

### 2. Start the frontend on the VM

Open a separate local terminal on the VM:

```bash
cd ~/tasc_science
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch science_bringup sci_frontend.launch.py
```

The frontend starts the keyboard teleop, robot visualization, and RViz.

## Debug / Verify the connection

From the VM, verify that the Jetson controller manager is visible:

```bash
ros2 node list
```

Check the science controllers:

```bash
ros2 control list_controllers \
  -c /science/controller_manager
```

Expected active controllers:

```text
joint_state_broadcaster
science_velocity_controller
beaker_position_controller
```

Check joint-state communication:

```bash
ros2 topic echo /science/joint_states
```

Check the velocity-controller command topic:

```bash
ros2 topic info \
  /science/science_velocity_controller/commands \
  --verbose
```

Check the beaker-controller command topic:

```bash
ros2 topic info \
  /science/beaker_position_controller/commands \
  --verbose
```

The command topics should show a publisher on the VM and a subscriber on the Jetson.

## Hardware notes

- The ESP32 serial device path must match the device connected to the Jetson
  - (scienceUSB, found in ```science_2026_system.hpp```).
- The Phidgets serial number, hub port, and channel must match the physical controller
  - (found in ```science_2026_system.hpp```).
- The ESP32 and servo power supply must share a common ground.
- The beaker servo is commanded to discrete positions of 0°, 90°, and 180°.
- The backend should be launched only on the computer physically connected to the hardware
  - (If testing while esp32 + VINT hub connected to laptop, launch from laptop).
- RViz should run on the VM rather than through the Jetson SSH session.
