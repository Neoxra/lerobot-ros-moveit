# SO-101 ROS 2 Integration

ROS 2 packages for controlling the SO-ARM101 robotic arm with ros2_control and MoveIt 2.

## Packages

- **so101_hardware**: C++ hardware interface plugin for Feetech STS3215 servos
- **so101_moveit**: MoveIt 2 configuration for motion planning and keyboard teleoperation
- **so101_description**: URDF robot description

## Quick Start

### 1. Install Dependencies

```bash
# Install ROS 2 Jazzy
# Follow: https://docs.ros.org/en/jazzy/Installation.html

# Install dependencies
cd ~/le_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 2. Build Packages

```bash
cd ~/le_ws
colcon build --packages-select so101_hardware so101_moveit so101_description
source install/setup.bash
```

### 3. Calibrate Robot

```bash
# Activate conda environment (for LeRobot calibration tool)
conda activate lerobot_ros

# Find robot port
lerobot-find-port

# Run calibration (follow on-screen instructions)
cd ~/le_ws
./calibrate_and_install.sh my_follower /dev/ttyACM1
```

This creates:
- `~/.cache/huggingface/lerobot/calibration/robots/so101_follower/my_follower.json`
- `~/le_ws/src/lerobot-ros/so101_moveit/config/motor_calibration.yaml`

### 4. Launch MoveIt with Real Hardware

```bash
# Find robot port (may change when unplugged)
lerobot-find-port

# Launch (adjust port as needed)
ros2 launch so101_moveit demo.launch.py port:=/dev/ttyACM1
```

This launches:
- C++ hardware interface (100 Hz control loop)
- ros2_control with 3 controllers
- MoveIt motion planning
- RViz visualization

### 5. Verify System

```bash
# Check controllers are active
ros2 control list_controllers
# Expected:
# arm_controller          [active]
# gripper_controller      [active]
# joint_state_broadcaster [active]

# Monitor joint states (should be 100 Hz)
ros2 topic hz /joint_states
```

## Features

### Hardware Interface (so101_hardware)
- Pure C++ implementation (no Python/GIL issues)
- Fast controller loading (< 1 second)
- 100 Hz real-time control loop
- URDF-limit-aware motor calibration
- Automatic torque management
- Uses [SCServo_Linux](https://github.com/adityakamath/SCServo_Linux) SDK for Feetech servos

### MoveIt Configuration (so101_moveit)
- Motion planning with OMPL
- Planning groups: `arm` (5-DOF) and `gripper`
- Named poses: `home`, `open`, `close`
- Velocity and acceleration limits configured
- Full RViz visualization

### Keyboard Teleoperation
Simple keyboard control for joint positions:

```bash
# Terminal 1: Launch hardware interface
source install/setup.bash
ros2 launch so101_moveit demo.launch.py port:=/dev/ttyACM1

# Terminal 2: Run keyboard teleoperation
source install/setup.bash
python3 src/lerobot-ros/so101_moveit/scripts/keyboard_teleop.py
```

**Controls:**
- Q/A - Joint 1 (shoulder pan) - decrease/increase
- W/S - Joint 2 (shoulder lift) - decrease/increase
- E/D - Joint 3 (elbow flex) - decrease/increase
- R/F - Joint 4 (wrist flex) - decrease/increase
- T/G - Joint 5 (wrist roll) - decrease/increase
- O/L - Gripper - close/open
- ESC - Exit

**Features:**
- **Multi-key support** - Hold multiple keys simultaneously for combined motion
- **Fast and responsive** - 30 Hz update rate for smooth control
- **Accumulator pattern** - Target positions accumulate for continuous movement
- **Joint limit enforcement** - Automatically clamps movements to URDF limits
- **Real-time position display** - Shows commanded joint positions in terminal
- **Separate gripper control** - Uses GripperActionController (different move group)
- **No terminal echo** - Clean output without key character spam
- **Proper ROS 2 node** - Uses standard ROS 2 patterns
- Step size: 0.04 rad (~2.3 degrees) for arm, 0.10 rad for gripper
- Trajectory duration: 0.15 seconds for smooth motion

## Architecture

```
┌─────────────────────────┐
│  MoveIt 2               │  Motion planning, RViz
└───────────┬─────────────┘
            │
            ↓
┌─────────────────────────┐
│  ros2_control           │  Controller management
└───────────┬─────────────┘
            │
            ↓
┌─────────────────────────┐
│  so101_hardware         │  C++ hardware interface plugin
└───────────┬─────────────┘
            │
            ↓
┌─────────────────────────┐
│  SCServo SDK            │  Serial communication (115200 baud)
└───────────┬─────────────┘
            │
            ↓
┌─────────────────────────┐
│  Feetech STS3215        │  6 servo motors
│  Servos                 │  (IDs 1-6)
└─────────────────────────┘
```

## Motor Calibration

The hardware interface maps raw motor positions (0-4095) to radians using URDF joint limits:

```cpp
// For arm joints:
progress = (raw_position - range_min) / (range_max - range_min)
radians = progress * (urdf_upper - urdf_lower) + urdf_lower

// For gripper:
normalized = (raw_position - range_min) / (range_max - range_min)  // [0, 1]
```

**Benefits:**
- Joint angles never exceed URDF limits
- Full motor range maps to full URDF range
- Each joint has different physical limits
- Calibration captures actual hardware constraints

**Joint Mapping:**

| ROS 2 Joint   | Motor ID | URDF Limits (rad) |
|---------------|----------|-------------------|
| shoulder_pan  | 1        | [-1.92, 1.92]     |
| shoulder_lift | 2        | [-1.75, 1.75]     |
| elbow_flex    | 3        | [-1.75, 1.57]     |
| wrist_flex    | 4        | [-1.66, 1.66]     |
| wrist_roll    | 5        | [-2.79, 2.79]     |
| gripper       | 6        | [0, 1] (normalized) |

## Launch Parameters

```bash
ros2 launch so101_moveit demo.launch.py \
  port:=/dev/ttyACM1 \
  use_fake_hardware:=false \
  calibration_file:=/path/to/motor_calibration.yaml
```

**Parameters:**
- `port`: Serial port for servo bus (default: `/dev/ttyACM0`)
- `use_fake_hardware`: Use fake controllers for testing (default: `false`)
- `calibration_file`: Path to motor calibration YAML (default: package config)

## Troubleshooting

### Serial Port Permissions

```bash
# Add user to dialout group (permanent)
sudo usermod -a -G dialout $USER
# Log out and back in

# Or temporarily:
sudo chmod 666 /dev/ttyACM1
```

### Port Changes

Robot port may change when unplugged/replugged. Always verify:
```bash
lerobot-find-port
# or
ls -la /dev/ttyACM*
```

### Motors Not Responding

1. Check power supply is ON
2. Verify port: `lsof /dev/ttyACM1`
3. Check motor IDs are 1-6
4. Try power cycling the robot
5. Check serial cable connection

### Joint States Not Publishing

1. Verify correct port: `ls /dev/ttyACM*`
2. Check hardware activated: `ros2 control list_hardware_interfaces`
3. Check controllers loaded: `ros2 control list_controllers`
4. View hardware logs for connection errors
5. Ensure robot is powered on before launching

### RViz Doesn't Match Physical Robot

1. Re-run calibration: `./calibrate_and_install.sh`
2. Verify `motor_calibration.yaml` has correct `range_min`/`range_max`
3. Rebuild: `colcon build --packages-select so101_hardware so101_moveit`
4. Ensure robot was moved through FULL range during calibration

### Controllers Timeout

- Old Python implementation had GIL threading issues
- Current C++ implementation loads in < 1 second
- If issues persist, check for conflicting processes: `lsof /dev/ttyACM1`

## Development

### Building from Source

```bash
cd ~/le_ws
colcon build --packages-select so101_hardware so101_moveit --cmake-clean-first
source install/setup.bash
```

### Package Structure

```
lerobot-ros/
├── so101_hardware/              # C++ hardware interface
│   ├── include/so101_hardware/  # Header files
│   ├── src/                     # Implementation (so101_system.cpp)
│   ├── scservo_sdk/             # Feetech servo C++ SDK
│   └── CMakeLists.txt           # Build configuration
├── so101_moveit/                # MoveIt 2 configuration
│   ├── config/                  # Controllers, limits, calibration, RViz
│   ├── launch/                  # Launch files
│   ├── scripts/                 # Keyboard teleoperation
│   └── urdf/                    # Robot URDF with ros2_control
└── so101_description/           # Robot URDF description
    └── urdf/                    # URDF/Xacro files
```

### Dependencies

**C++ (ros2_control):**
- `hardware_interface`
- `pluginlib`
- `rclcpp`
- `rclcpp_lifecycle`
- SCServo SDK (included in `scservo_sdk/`)

**Python (keyboard teleoperation):**
- `pynput` - Keyboard input library

```bash
pip3 install pynput
```

## Resources

- [SO-101 Hardware Interface README](./so101_hardware/README.md) - Detailed hardware documentation
- [LeRobot Documentation](https://huggingface.co/docs/lerobot/index)
- [SO-101 Tutorial](https://huggingface.co/docs/lerobot/so101)
- [ros2_control Documentation](https://control.ros.org/)
- [MoveIt 2 Documentation](https://moveit.ai/)
- [SCServo_Linux GitHub](https://github.com/adityakamath/SCServo_Linux)

## License

Apache License 2.0 - Copyright 2024 The HuggingFace Inc. team
