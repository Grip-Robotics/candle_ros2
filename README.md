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
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch candle_ros2 md_node_launch.py \
  init_devices_zero:=false
```

In terminal 2, source the workspace and run the remaining commands in order:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

1. Add and initialize all drives in impedance mode:

```bash
ros2 service call /md/init_devices candle_ros2/srv/InitDevices \
  "{device_ids: [342, 343, 345], mode: 'IMPEDANCE'}"
```

With `startup_position_policy:=restore_or_home`, each drive either restores its
trusted host-side state or is queued for open-stop homing. Homing is
asynchronous, so the initialization response can be `false` while homing is
running. Wait for the `homing complete` log before commanding that drive. If you want to home every time, set `startup_position_policy:=always_home`.

1. For a deliberate manual calibration instead of automatic homing, place the

requested drives at the mechanical open reference and call zero. This aborts
active homing, defines logical `0`, and atomically updates the host-side state
file without writing the drive's flash:

```bash
ros2 service call /md/zero candle_ros2/srv/Generic \
  "{device_ids: [342, 343, 345]}"
```

To retry automatic homing explicitly:

```bash
ros2 service call /md/home candle_ros2/srv/Generic \
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

The requested drives are configured sequentially over CAN and then move
concurrently toward logical zero in `POSITION_PROFILE` mode. Opening uses the
configured `6 rad/s` velocity, `3 Nm` torque limit, and dedicated acceleration
and deceleration parameters. Both the position and velocity PID Kp registers
must already be configured; otherwise that drive remains disabled and its
service result is false. A raw target that would move opposite the logical
opening direction is rejected and faults the position reference.

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

The MD main encoder retains its single-turn reference after a drive reset, while
its accumulated gearbox turn section is lost. For the 10:1 gripper drive this produces
position jumps of approximately `2π / 10 = 0.62831853 rad`. While the drive
remains powered, its accumulated MD position is already continuous, so normal
tracking applies one fixed raw-to-logical offset even across sparse, fast
samples. Nearest-wrap selection is used only during explicit communication
recovery and persisted startup restore, where the accumulated-turn component
may actually have been lost.

On a CAN outage the node publishes `NaN` joint-state values, rejects new motion
commands, retries communication every `25 ms`, and requires three healthy
position/status samples. Recovery succeeds only when inferred unpowered motion
is at most `0.25 rad`. The interrupted target is then resumed automatically;
an interrupted soft-close resumes its final closed target with the slow
impedance configuration.

If recovery is ambiguous, the drive remains disabled. Place it manually at the
mechanical open reference and call `/md/zero`.

### Drive health

The node publishes stack-compatible per-drive health on `/md/health` every
`health_publish_period_ms` (default `200 ms`):

```bash
ros2 topic echo /md/health
```

All `MdHealth` arrays use the same `device_ids` ordering. `responsive` is false
only when a drive does not answer its status query. `error` is true for
unavailable status, a real drive error, or a position state that rejects
commands. Warning-only firmware flags are listed in `active_errors` without
setting `error`.

Position states are reported through `active_errors` without changing the
message contract used by the larger stack. `TRACKING` adds no position error;
`UNINITIALIZED`, `RESTORING`, `HOMING`, `RECOVERING`, and `FAULTED` set
`error=true` and describe why commands are rejected.

Per-gripper operational state is published separately, with arrays aligned by
device ID:

```bash
ros2 topic echo /md/gripper_state
```

The reported state is `HOMING` while homing is queued or active, `UNKNOWN` when
position tracking or telemetry is unavailable, and `MOVING` while measured
velocity exceeds `gripper_state_moving_velocity_rad_s`. A stopped gripper near
its configured endpoint is `OPEN` or `CLOSED`; a stopped gripper between those
endpoints is `IDLE`. This topic reuses the regular joint-state samples and does
not add CAN reads.

### Persistent startup restore and homing

The node stores the last trusted logical position atomically in
`~/.ros/candle_ros2_position_state.json`. On `/md/init_devices`, it keeps each
drive disabled, validates three stable encoder/status samples, and restores the
nearest wrap branch when it is within `0.25 rad` of the saved state.

With the default `startup_position_policy:=restore_or_home`, a missing, corrupt,
or incompatible state queues automatic open-stop homing. Homing is performed
one drive at a time: a small initial backoff is followed by a ramped low-torque
seek, another backoff, and a slower second seek. Both stop positions must agree.
The drive is then zeroed, persisted to the host-side state file, returned to
impedance mode, and held at logical zero. Manual retry is available with:

```bash
source install/setup.bash

ros2 service call /md/home candle_ros2/srv/Generic \
  "{device_ids: [342]}"
```

Homing intentionally contacts the mechanical open stop. Verify
`homing_direction_by_id` and begin with low torque. Any CAN loss, low bus
voltage, encoder/status fault, excessive velocity/travel, timeout, or
non-repeatable stop leaves the drive disabled.

The JSON restore is still assumption-based. Movement close to one complete
`0.628 rad` encoder wrap while the drive and node are both off cannot be
distinguished without an output-side absolute encoder or independent home
sensor.

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
2. Closes and opens each gripper individually for five cycles.
3. Soft-closes each gripper for five cycles with `40`, `30`, and `25` mm
  transition gaps, reopening after every movement.

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

The default waits are one second after normal motion and two seconds after a
soft close. Movement cycles, delays, and service timeout can be overridden:

```bash
MOVEMENT_CYCLES=3 \
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
- `gripper_velocity_limit_rad_s` (`6.0`)
- `gripper_torque_limit_nm` (`3.0`)
- `opening_profile_acceleration_rad_s2` / `opening_profile_deceleration_rad_s2` (`100.0` / `100.0`)
- `soft_close_fast_torque_limit_nm` / `soft_close_slow_torque_limit_nm` (`4.0` / `4.0`)
- `soft_close_profile_acceleration_rad_s2` / `soft_close_profile_deceleration_rad_s2` (`100.0` / `100.0`)
- `soft_close_closed_tol_rad` (`0.005`)
- `soft_close_fast_duration_ms` (`300`) — slow stage starts this long after the request
- `health_publish_period_ms` (`200`)
- `gripper_state_position_tolerance_rad` (`0.025`)
- `gripper_state_moving_velocity_rad_s` (`0.12`)
- `encoder_wrap_period_rad` (`0.62831853`)
- `position_recovery_max_delta_rad` (`0.25`, must be less than half the wrap period)
- `position_recovery_samples` (`3`)
- `position_recovery_retry_ms` (`25`)
- `startup_position_policy` (`restore_or_home`; alternatives: `restore_only`, `always_home`)
- `position_state_file` (`~/.ros/candle_ros2_position_state.json`)
- `position_state_write_period_ms` (`1000`)
- `position_state_min_change_rad` (`0.005`)
- `homing_direction_by_id` (`342:-1,343:-1,345:-1`)
- `homing_torque_nm` / `homing_second_pass_torque_nm` (`0.3` / `0.3`)
- `homing_torque_ramp_ms` (`500`)
- `homing_velocity_trip_rad_s` (`6.0`)
- `homing_min_bus_voltage_v` (`10.0`)
- `homing_stall_velocity_rad_s` / `homing_stall_dwell_ms` (`0.06` / `250`)
- `homing_max_travel_rad` / `homing_timeout_ms` (`0.75` / `5000`)
- `homing_backoff_rad` / `homing_repeatability_rad` (`0.03` / `0.015`)
- `homing_min_motion_rad` (`0.01`)
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
profile velocity, and `soft_close_slow_torque_limit_nm`. The node does not
modify the drive's firmware maximum-position or maximum-velocity registers.

## Documentation

Full CANdle ROS2 documentation:
➡️ [CANdle ROS2 nodes documentation](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/CANdle_ROS2/intro.html)

MAB controllers manuals:
➡️ [MAB documentation](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/intro.html)