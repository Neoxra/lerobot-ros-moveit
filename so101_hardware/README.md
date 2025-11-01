# SO-101 Hardware Interface

ros2_control hardware interface plugin for the SO-101 robot with Feetech STS3215 servos.

## Overview

This package provides a hardware interface plugin that integrates LeRobot's FeetechMotorsBus with ros2_control. It uses Python/C API to embed Python functionality within the C++ plugin, allowing direct use of the tested LeRobot motor control code.

## Architecture

- **C++ Plugin**: Implements `hardware_interface::SystemInterface` for ros2_control
- **Python Integration**: Embeds Python interpreter to call `lerobot.motors.feetech.FeetechMotorsBus`
- **Motor Mapping**: Maps ROS 2 joint names to LeRobot motor names

### Joint Mapping

| ROS 2 Joint Name | LeRobot Motor Name | Motor ID | Servo Model |
|------------------|-------------------|----------|-------------|
| Rotation         | shoulder_pan      | 1        | sts3215     |
| Pitch            | shoulder_lift     | 2        | sts3215     |
| Elbow            | elbow_flex        | 3        | sts3215     |
| Wrist_Pitch      | wrist_flex        | 4        | sts3215     |
| Wrist_Roll       | wrist_roll        | 5        | sts3215     |
| Jaw              | gripper           | 6        | sts3215     |

## Usage

### In URDF/Xacro

```xml
<xacro:so101_new_calib_ros2_control
  name="SO101SystemHardware"
  initial_positions_file="${initial_positions_file}"
  use_fake_hardware="false"
  port="/dev/ttyACM0"/>
```

Parameters:
- `use_fake_hardware`: Set to `false` to use real hardware
- `port`: Serial port for Feetech servo bus (default: `/dev/ttyACM0`)

### Launch with MoveIt

```bash
# Terminal 1: Launch robot with real hardware
ros2 launch so101_moveit demo.launch.py use_fake_hardware:=false

# Or specify custom port
ros2 launch so101_moveit demo.launch.py use_fake_hardware:=false port:=/dev/ttyACM0
```

## Features

- **Automatic Python Integration**: Initializes Python interpreter and imports LeRobot modules
- **Motor Connection**: Connects to Feetech servo bus on configure
- **Torque Management**: Enables torque on activate, disables on deactivate
- **Position Control**: Reads and writes motor positions
- **Unit Conversion**: Automatically converts between radians (ros2_control) and degrees (Feetech)
- **Initial Positions**: Loads joint initial positions from YAML config file

## Initial Positions Configuration

The hardware interface loads initial joint positions from a YAML configuration file. This file is specified in the xacro macro:

```yaml
# Example: initial_positions.yaml
initial_positions:
  Rotation: 0.0
  Pitch: 0.0
  Elbow: 0.0
  Wrist_Pitch: 0.0
  Wrist_Roll: 0.0
  Jaw: 0.0
```

These values are:
- Loaded during `on_init()` from the `initial_value` parameter in each joint's state interface
- Used to initialize the hardware state
- Logged at startup for verification
- **Safety**: On activation, the interface reads the actual motor positions and uses those (not the initial positions) to prevent sudden movements

## Implementation Details

### Lifecycle States

1. **on_init**:
   - Validates joint interfaces and stores port parameter
   - Loads initial positions from config file
   - Logs configured initial positions for each joint

2. **on_configure**:
   - Initializes Python, creates FeetechMotorsBus
   - Connects to motors
   - Sets up initial state from config

3. **on_activate**:
   - Reads current motor positions
   - Logs actual motor positions
   - Enables motor torque
   - Sets commands to current positions (safety)

4. **read**: Reads motor positions via `motor_bus.sync_read("Present_Position")`

5. **write**: Writes motor commands via `motor_bus.sync_write("Goal_Position")`

6. **on_deactivate**: Disables motor torque

### Python Dependencies

The hardware interface requires:
- Python 3.10+
- `lerobot` package installed and accessible
- `scservo_sdk` (Feetech servo SDK)

The Python path is hardcoded to `/home/nick-bell/le_ws/src/lerobot/src`. If your workspace is in a different location, update `so101_system.cpp:56`.

## Troubleshooting

### Build Warnings

The build shows a deprecation warning about `on_init()` signature. This can be safely ignored for now and will be fixed in a future update to use the newer API.

### Python Import Errors

If you see Python import errors:
1. Ensure lerobot is installed: `pip install -e /home/nick-bell/le_ws/src/lerobot`
2. Check that scservo_sdk is installed: `pip install scservo_sdk`
3. Verify the Python path in `so101_system.cpp` matches your workspace

### Serial Port Permissions

If you get permission denied errors:
```bash
sudo chmod 666 /dev/ttyACM0
# Or add user to dialout group:
sudo usermod -a -G dialout $USER
# Then log out and log back in
```

### Motor Connection Issues

- Verify the correct port with `lerobot-find-port`
- Check that motors are powered on
- Ensure motor IDs are correctly configured (1-6)
- Try with `handshake=False` in connect (already set by default)

## Development

### Building

```bash
cd /home/nick-bell/le_ws
colcon build --packages-select so101_hardware
source install/setup.bash
```

### Testing

Test with controller_manager:
```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

## License

Apache License 2.0 - Copyright 2024 The HuggingFace Inc. team
