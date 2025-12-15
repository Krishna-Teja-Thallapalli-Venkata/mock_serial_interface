# Mock Serial Hardware Interface

A ROS 2 package implementing a `ros2_control` hardware interface with a simulated serial motor driver.

## Prerequisites

- **ROS 2 Version**: Jazzy Jalisco (tested on Ubuntu 24.04)
- **Dependencies**: `ros2_control`, `ros2_controllers`, `xacro`

## Installation

1.  Clone this repository into your ROS 2 workspace (e.g., `~/ros2_ws/src`):
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone <this_repo>
    ```
    *Note: If you are using the `robotics_assignment` folder as your workspace root:*
    ```bash
    cd ~/robotics_assignment
    ```

2.  Install dependencies:
    ```bash
    sudo apt update
    sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-xacro
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  Build the workspace:
    ```bash
    colcon build --symlink-install
    ```

4.  Source the setup file:
    ```bash
    source install/setup.bash
    ```

## Usage

### 1. Launch the Mock Robot

This will launch the `ros2_control` node, the mock hardware interface, and Rviz.

```bash
ros2 launch mock_serial_interface test_mock_hardware.launch.py
```

### 2. Verify Operation

**Check Hardware Interface:**
Open a new terminal and run:
```bash
ros2 control list_hardware_interfaces
```
You should see `MockRobot/joint1` available.

**Send a Command:**
Move the joint by publishing a trajectory command:

```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{joint_names: ["joint1"], points: [{positions: [1.57], time_from_start: {sec: 2}}]}' -1
```

You should see the mock driver logs in the launch terminal, and the robot moving in Rviz.

## Features

- **MockDriver**: Simulates a serial device with a byte-based protocol.
    - Protocol: `[HEADER, CMD, VAL_H, VAL_L]`
    - Simulates motor dynamics (velocity ramp).
- **MockHardwareInterface**:
    - Implements `SystemInterface`.
    - Reads/Writes packets to the mock driver.
