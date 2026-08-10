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



## Example MD service calls - GRIPPER CONTROL



### Calibrated three-drive soft close

The following commands are the calibrated sequence for drives `342`, `343`, and
`345`. Encoder zeros are preserved during normal initialization. Zero a drive
only as a deliberate calibration step while it is at its known mechanical open
reference.

In terminal 1, launch the node and keep it running:

```bash
ros2 launch candle_ros2 md_node_launch.py \
  soft_close_fast_velocity_rad_s:=10.0 \
  soft_close_slow_velocity_rad_s:=2.6 \
  soft_close_fast_torque_limit_nm:=4.0 \
  soft_close_slow_torque_limit_nm:=4.0 \
  soft_close_profile_acceleration_rad_s2:=20.0 \
  soft_close_profile_deceleration_rad_s2:=30.0 \
  init_devices_zero:=false
```

In terminal 2, source the workspace and run the remaining commands in order:

```bash
source install/setup.bash
```

1. Add, configure, and enable all drives in impedance mode without changing
   their encoder zeros:

```bash
ros2 service call /md/init_devices candle_ros2/srv/InitDevices \
  "{device_ids: [342, 343, 345], mode: 'IMPEDANCE'}"
```

2. With every gripper at its known mechanical open reference, zero all drives:

```bash
ros2 service call /md/zero candle_ros2/srv/Generic \
  "{device_ids: [342, 343, 345]}"
```

3. Apply the calibrated runtime position and velocity PID gains:

```bash
ros2 topic pub --once /md/position_command candle_ros2/msg/PositionPidCmd \
  "{device_ids: [342, 343, 345],
    position_pid: [
      {kp: 12.5, ki: 0.5, kd: 0.05, i_windup: 1.0, max_output: 10.0},
      {kp: 12.5, ki: 0.5, kd: 0.05, i_windup: 1.0, max_output: 10.0},
      {kp: 12.5, ki: 0.5, kd: 0.05, i_windup: 1.0, max_output: 10.0}
    ],
    velocity_pid: [
      {kp: 1.5, ki: 0.02, kd: 0.0, i_windup: 1.0, max_output: 4.0},
      {kp: 1.5, ki: 0.02, kd: 0.0, i_windup: 1.0, max_output: 4.0},
      {kp: 1.5, ki: 0.02, kd: 0.0, i_windup: 1.0, max_output: 4.0}
    ]}"
```

4. Open all grippers:

```bash
ros2 service call /md/open_gripper candle_ros2/srv/Generic \
  "{device_ids: [342, 343, 345]}"
```

5. Run the calibrated two-stage close for one drive at a time. Adjust
   `pre_close_gap_mm` when a different transition gap is required:

```bash
ros2 service call /md/soft_close_gripper candle_ros2/srv/SoftCloseGripper \
  "{device_ids: [342], pre_close_gap_mm: 3.0}"
```

6. Or close one drive all the way using impedance mode:

```bash
ros2 service call /md/close_gripper candle_ros2/srv/Generic \
  "{device_ids: [342]}"
```

The PID command above changes runtime registers and may need to be repeated
after a drive reset or power cycle. Never use `init_devices_zero:=true` unless
all mechanisms are physically at the intended zero reference.

### Automated service test

The installed test script runs the complete calibrated procedure for drives
`342`, `343`, and `345`. Build and source the package, keep the calibrated node
launch above running in another terminal, then execute:

```bash
colcon build --packages-select candle_ros2
source install/setup.bash
ros2 run candle_ros2 test_grippers.sh
```

By default, the script preserves existing encoder zeros and requires typing
`RUN` before moving. After confirmation, it automatically:

1. Applies the calibrated position and velocity PID gains.
2. Closes and opens each gripper individually.
3. Soft-closes each gripper individually with `40`, `30`, and `20` mm
   transition gaps, reopening after every test.

The soft-close values are the gaps where motion changes from fast to slow; they
are not final commanded widths. The script intentionally avoids simultaneous
closing because the combined current draw can cause an undervoltage fault.

Use `--zero` only for deliberate encoder calibration after placing every
mechanism at its mechanical open reference:

```bash
ros2 run candle_ros2 test_grippers.sh --zero
```

Use `--yes` only when it is safe to skip the interactive motion confirmation:

```bash
ros2 run candle_ros2 test_grippers.sh --yes
```

Motion delays and service timeout can be overridden when slower hardware needs
more time:

```bash
MOVE_WAIT_SECONDS=3 \
SOFT_CLOSE_WAIT_SECONDS=9 \
SERVICE_TIMEOUT_SECONDS=20 \
ros2 run candle_ros2 test_grippers.sh
```

If a service reports failure, times out, or the script is interrupted, the
script requests `/md/disable` for all three drives.

### Additional gripper commands

Set impedance gains explicitly when needed; normal open/close writes the
node-wide impedance settings again:

```bash
ros2 topic pub /md/impedance_command candle_ros2/msg/ImpedanceCmd \
  "{device_ids: [343], kp: [12.5], kd: [0.05], max_output: [4.0]}" --once

# Three independently calibrated motors in one acknowledged batch request
ros2 service call /md/configure_gripper candle_ros2/srv/ConfigureGripper \
  "{device_ids: [342, 343, 345], kp: [12.5, 12.5, 12.5], kd: [0.05, 0.05, 0.05],
    velocity_limit_rad_s: [3.5, 3.5, 3.5],
    torque_limit_nm: [4.0, 4.0, 4.0]}"
ros2 service call /md/set_gripper_targets candle_ros2/srv/SetGripperTargets \
  "{device_ids: [342, 343, 345], target_position_rad: [0.62, 0.62, 0.62]}"
```

The legacy open/close services use the node-wide gripper parameters. A
multi-motor gripper with independently calibrated targets should use
`/md/set_gripper_targets`, which accepts one target and limit set per drive.

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

```

Soft close uses the position and velocity PID gains stored in the drive. The
service rejects the request when either position Kp or velocity Kp is not
configured. It applies hard position limits between the configured open and
closed positions and does not use target overtravel.

## Documentation

Full CANdle ROS2 documentation:
➡️ [CANdle ROS2 nodes documentation](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/CANdle_ROS2/intro.html)

MAB controllers manuals:
➡️ [MAB documentation](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/intro.html)
```

