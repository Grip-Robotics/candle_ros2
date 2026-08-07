# CANdle ROS2

This repository provides ROS2 interfaces for controlling **MD electric drive controllers** and **PDS power distribution systems** using the [CANdle-SDK](https://github.com/mabrobotics/CANdle-SDK).
It exposes both systems as ROS2 nodes with services and topics for operational control and telemetry.

This package acts as a **runtime control interface**.
For configuration, please use:

➡️ [CANdleTool](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/CANdle-SDK/CANdleTool.html)

## Features

### MD Node
- Control MD drive controllers
- Publish joint state data
- Accept position, velocity, motion, and impedance commands
- Provide enable/disable/zero/mode setup services

### PDS Node
- Manage PDS devices and their modules
- Monitor modules such as the Control Board, Isolated Converter, Brake Resistor, and Power Stage

## Installation

Go to your ROS2 workspace and clone the repository:

```bash
git clone git@github.com:mabrobotics/candle_ros2.git src/candle_ros2
```

Initialize submodules:

```bash
git -C src/candle_ros2/ submodule update --init --recursive
```

Build:

```bash
colcon build
```

Source the environment:

```bash
source install/setup.bash
```

## Running

### MD Node
```bash
ros2 launch candle_ros2 md_node_launch.py
```

### PDS Node
```bash
ros2 launch candle_ros2 pds_node_launch.py
```

### Both Nodes
```bash
ros2 launch candle_ros2 both_launch.py
```

### Launch arguments

- `bus` — desired communication bus with CANdle device, possible values: `USB` and `SPI` (default: `USB`).
- `data_rate` — data rate of CAN network, possible values: `1M`, `2M`, `5M` and `8M` (default: `1M`).
- `default_qos` — ROS message quality of service for node's publishers, possible values: `Reliable` and `BestEffort` (default: `Reliable`).

Example launch command with custom arguments:
```bash
ros2 launch candle_ros2 md_node_launch.py bus:=SPI data_rate:=5M
```

## Example MD service calls - GRIPPER CONTROL

Bring up one or more drives, then open or close the gripper. `init_devices`
adds each drive, applies the requested mode, and enables it. Encoder zeroing is
disabled by default: zero only when the mechanism is at a known reference, or
launch with `init_devices_zero:=true` when that condition is guaranteed.

The legacy open/close services use the node-wide gripper parameters. A
multi-motor gripper should instead use `/md/set_gripper_targets`, which accepts
one independently calibrated target and limit set per drive.

```bash
# Bring up device 343 in impedance mode
ros2 service call /md/init_devices candle_ros2/srv/InitDevices \
  "{device_ids: [343], mode: 'IMPEDANCE'}"

# Close / open gripper
ros2 service call /md/close_gripper candle_ros2/srv/Generic "{device_ids: [343]}"
ros2 service call /md/open_gripper candle_ros2/srv/Generic "{device_ids: [343]}"

# Soft close: POSITION_PROFILE fast move, then slow profile after 500 ms
ros2 service call /md/soft_close_gripper candle_ros2/srv/SoftCloseGripper \
  "{device_ids: [343], pre_close_gap_mm: 25.0}"

# Soft-close tuning example; the next /md/init_devices call also zeros the drive
ros2 launch candle_ros2 md_node_launch.py \
  soft_close_fast_velocity_rad_s:=10.0 \
  soft_close_slow_velocity_rad_s:=2.6 \
  soft_close_fast_torque_limit_nm:=4.0 \
  soft_close_slow_torque_limit_nm:=4.0 \
  soft_close_profile_acceleration_rad_s2:=20.0 \
  soft_close_profile_deceleration_rad_s2:=30.0 \
  init_devices_zero:=true

# Optional: set impedance gains explicitly (overwritten again by open/close)
ros2 topic pub /md/impedance_command candle_ros2/msg/ImpedanceCmd \
  "{device_ids: [343], kp: [12.5], kd: [0.05], max_output: [4.0]}" --once

# Three independently calibrated motors in one acknowledged batch request
ros2 service call /md/configure_gripper candle_ros2/srv/ConfigureGripper \
  "{device_ids: [343, 344, 345], kp: [12.5, 12.5, 12.5], kd: [0.05, 0.05, 0.05],
    velocity_limit_rad_s: [3.5, 3.5, 3.5],
    torque_limit_nm: [4.0, 4.0, 4.0]}"
ros2 service call /md/set_gripper_targets candle_ros2/srv/SetGripperTargets \
  "{device_ids: [343, 344, 345], target_position_rad: [0.62, 0.62, 0.62]}"

# Home at the mechanical open endstop (gentle open → zero → restore gains → save)
# Unlike /md/zero, this persists the zero (and restored config) across power cycles.
ros2 service call /md/home_gripper candle_ros2/srv/HomeGripper \
  "{device_ids: [343, 344, 345]}"
```

Individual steps are also available as `/md/add_mds`, `/md/set_mode`, `/md/zero`, and `/md/enable`.

Relevant MD-node parameters are:

- `joint_name_prefix` (`md_`)
- `gripper_open_position_rad` (`0.0`)
- `gripper_closed_position_rad` (`0.62`)
- `gripper_open_gap_mm` / `gripper_closed_gap_mm` (`120.0` / `0.0`) — measured finger gap at open/close; used by soft close
- `gripper_impedance_kp` / `gripper_impedance_kd` (`12.5` / `0.05`)
- `gripper_velocity_limit_rad_s` (`3.5`)
- `gripper_torque_limit_nm` (`4.0`)
- `soft_close_fast_velocity_rad_s` / `soft_close_slow_velocity_rad_s` (`2.0` / `0.3`)
- `soft_close_fast_torque_limit_nm` / `soft_close_slow_torque_limit_nm` (`3.0` / `3.0`)
- `soft_close_profile_acceleration_rad_s2` / `soft_close_profile_deceleration_rad_s2` (`5.0` / `5.0`)
- `soft_close_closed_tol_rad` (`0.005`)
- `soft_close_fast_duration_ms` (`500`) — slow stage starts this long after the request
- `init_devices_zero` (`false`)
- `home_impedance_kp` / `home_impedance_kd` (`4.0` / `0.05`)
- `home_torque_limit_nm` (`2.5`)
- `home_velocity_limit_rad_s` (`1.0`)
- `home_step_rad` (`0.05`)
- `home_stall_velocity_rad_s` (`0.02`)
- `home_stall_position_eps_rad` (`0.005`)
- `home_stall_torque_nm` (`0.25`)
- `home_stall_hold_ms` (`300`)
- `home_timeout_ms` (`8000`)
- `home_poll_ms` (`20`)

Soft close uses the position and velocity PID gains stored in the drive. The
service rejects the request when either position Kp or velocity Kp is not
configured. It applies hard position limits between the configured open and
closed positions and does not use target overtravel.

## Documentation

Full CANdle ROS2 documentation:
➡️ [CANdle ROS2 nodes documentation](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/CANdle_ROS2/intro.html)

MAB controllers manuals:
➡️ [MAB documentation](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/intro.html)
