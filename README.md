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
reference. Before starting this sequence, place every gripper at that mechanical
open reference.

In terminal 1, launch the node and keep it running:

```bash
ros2 launch candle_ros2 md_node_launch.py
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

1. With every gripper still at its known mechanical open reference, zero all
  drives. This command also replaces the old motion target with logical `0`.
  Runtime zero is not retained by the MD across a drive power reset:

```bash
ros2 service call /md/zero candle_ros2/srv/Generic \
  "{device_ids: [342, 343, 345]}"
```

1. (OPTIONAL) Apply the calibrated runtime position and velocity PID gains:

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

1. Open all grippers:

```bash
ros2 service call /md/open_gripper candle_ros2/srv/Generic \
  "{device_ids: [342, 343, 345]}"
```

1. Run the calibrated two-stage close for one drive at a time. Adjust
  `pre_close_gap_mm` when a different transition gap is required:

```bash
ros2 service call /md/soft_close_gripper candle_ros2/srv/SoftCloseGripper \
  "{device_ids: [342], pre_close_enabled: true, pre_close_gap_mm: 30.0,
    pre_close_offset_mm: 0.0,
    fast_speed: 1.0, slow_speed: 0.4}"
```

Set `pre_close_enabled: false` for one direct profile to the closed position.
In this mode `pre_close_gap_mm`, `pre_close_offset_mm`, and `slow_speed` do not
affect motion:

```bash
ros2 service call /md/soft_close_gripper candle_ros2/srv/SoftCloseGripper \
  "{device_ids: [342], pre_close_enabled: false, pre_close_gap_mm: 0.0,
    pre_close_offset_mm: 0.0,
    fast_speed: 1.0, slow_speed: 0.0}"
```

1. Or close one drive all the way using impedance mode:

```bash
ros2 service call /md/close_gripper candle_ros2/srv/Generic \
  "{device_ids: [342]}"
```

The PID command above changes runtime registers and may need to be repeated
after a drive reset or power cycle. Keep `init_devices_zero:=false` during
normal startup and fault recovery so re-initialization cannot redefine zero at
an arbitrary mechanism position. Use `/md/zero` only as a deliberate calibration
command while all requested mechanisms are physically at their intended open
reference.

### Brownout position continuity

The MD main encoder retains one motor revolution after a drive reset, while its
turn count and runtime zero are lost. For the 10:1 gripper drive this produces
position jumps of approximately `2π / 10 = 0.62831853 rad`. While this ROS node
remains running, it keeps a continuous logical position per drive and unwraps
such jumps against the last trusted position.

On a CAN outage the node publishes `NaN` joint-state values, rejects new motion
commands, retries communication every `25 ms`, and requires three healthy
position/status samples. Recovery succeeds only when inferred unpowered motion
is at most `0.25 rad`. The interrupted target is then resumed automatically;
an interrupted soft-close resumes its final closed target with the slow
impedance configuration.

If recovery is ambiguous, the drive remains disabled. Place it manually at the
mechanical open reference and call `/md/zero`. Continuity state is intentionally
not stored on disk, so a simultaneous MD and ROS-node restart also requires this
manual calibration.

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
3. Soft-closes each gripper individually with `40`, `30`, and `25` mm
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
- `gripper_closed_position_rad` (`0.63` from the launch file)
- `gripper_impedance_kp` / `gripper_impedance_kd` (`12.5` / `0.05`)
- `gripper_velocity_limit_rad_s` (`3.5`)
- `gripper_torque_limit_nm` (`4.0`)
- `soft_close_fast_torque_limit_nm` / `soft_close_slow_torque_limit_nm` (`4.0` / `4.0`)
- `soft_close_profile_acceleration_rad_s2` / `soft_close_profile_deceleration_rad_s2` (`100.0` / `100.0`)
- `soft_close_closed_tol_rad` (`0.005`)
- `soft_close_fast_duration_ms` (`300`) — slow stage starts this long after the request
- `encoder_wrap_period_rad` (`0.62831853`)
- `position_recovery_max_delta_rad` (`0.25`, must be less than half the wrap period)
- `position_recovery_samples` (`3`)
- `position_recovery_retry_ms` (`25`)
- `init_devices_zero` (`false`)

Soft-close adds the signed `pre_close_offset_mm` to `pre_close_gap_mm`, then
converts the resulting effective gap to motor position using piecewise-linear
interpolation between measured calibration points: `0 mm → 0.63 rad`,
`15 mm → 0.44 rad`, `30 mm → 0.366 rad`, `50 mm → 0.228 rad`, and
`90 mm → 0.05 rad`. Requested gaps outside `0`–`90 mm` are clamped to the
nearest endpoint.

The slow-stage monitoring timeout is `1000 ms`. Reaching the timeout ends the
software job, but leaves impedance control active and holding its target.

Soft-close speed fields are normalized request values in `[0, 1]`. They map
linearly to physical velocity using `velocity = 0.4 + speed * 5.6` rad/s, so
`0.0` means `0.4 rad/s` and `1.0` means `6.0 rad/s`. Requests outside this
range are rejected.

The fast soft-close stage uses the position and velocity PID gains stored in
the drive. The service rejects the request when either position Kp or velocity
Kp is not configured. The slow stage uses impedance control with
`gripper_impedance_kp`, `gripper_impedance_kd`, the requested slow speed as its
velocity limit, and `soft_close_slow_torque_limit_nm`. Soft close applies hard
position limits between the configured open and closed positions and does not
use target overtravel.

## Documentation

Full CANdle ROS2 documentation:
➡️ [CANdle ROS2 nodes documentation](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/CANdle_ROS2/intro.html)

MAB controllers manuals:
➡️ [MAB documentation](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/intro.html)