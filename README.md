# CANdle ROS2

ROS 2 interfaces for MD motor controllers and PDS power distribution systems,
using the bundled [CANdle-SDK](https://github.com/mabrobotics/CANdle-SDK).
One process owns the bus; each added MD has independent command state.

## Continuous impedance closing

Both `md/close_gripper` and `md/soft_close_gripper` now close in IMPEDANCE
mode from the first movement. There is no pre-close stage, gap estimate,
position-profile dependency, or timed fast-to-slow switch.

The driver starts with a target at the measured position. Each fresh feedback
sample advances it toward the calibrated closed endpoint at the requested
target ramp speed, bounded to `close_target_lead_rad` ahead of the measured
finger. The MD runs its own impedance PD controller with zero velocity and
torque feedforward and a torque cap throughout closing and holding.

For a large object touching the fingers immediately, the position target
therefore cannot continue advancing through the remaining sweep. The lead
bound applies before contact is detected. At rest, the spring contribution
to motor torque is at most approximately `kp * lead`; the configured torque
cap also applies. These are motor torques, not calibrated fingertip forces.

Contact is inferred when closing-direction torque exceeds the threshold,
measured speed is low, and the applied target is ahead of the finger for
the configured dwell. Then the target stops advancing. If an object pushes
the finger backward, the target retreats as needed to maintain the lead
bound. Losing contact does not restart closing. A new close request is
required to advance again. Reaching the closed endpoint is reported
separately from inferred contact.

This is an inferred contact detector: friction or a jam can resemble an
object. The default gains and thresholds require commissioning on the actual
mechanism. In particular, contact torque must be lower than both torque caps
and `kp * lead`.

### Timing and failures

Control reuses each drive's batched position/velocity/torque feedback. The
default timer is 50 ms. Delayed ticks advance the target by at most one
nominal period; they do not catch up with a large setpoint jump. Gaps above
`close_feedback_timeout_s`, invalid feedback, travel violations, and close
timeouts abort the job and request drive disable.

Contact and endpoint holds remain monitored until replaced or disabled.
A setup failure never falls back to enabling a stale target. Configuration
writes that could change gains or torque limits are rejected while closing
or holding; cancel the operation first.

Explicit disable cancels the job even if the hardware write fails. Bus
recovery cannot restart motion. After a fault or disable, explicit
`md/enable` or successful `md/init_devices` is required; enable establishes
a hold at the measured position. Keep the MD hardware communication watchdog
enabled: software cannot confirm a disable across a disconnected bus.

Services return per-drive **command startup acknowledgments**, not final
grasp results. Logs report `contact inferred` or `closed endpoint`.
Abort reasons are latched in `md/health` until successful enable/init.
Applications must not treat service acceptance as proof of a successful pick.

## Build and run

```bash
colcon build --packages-select candle_ros2
source install/setup.bash
ros2 launch candle_ros2 md_node_launch.py
```

PDS-only and combined launches remain `pds_node_launch.py` and
`both_launch.py`.

The node accepts `bus:=USB|SPI`, `data_rate:=1M|2M|5M|8M`,
`usb_serial:=SERIAL` (empty selects the first available adapter), and
`default_qos:=Reliable|BestEffort`. The MD launch exposes the settings below.

## Configuration

| Parameter | Default | Meaning |
| --- | --- | --- |
| `gripper_open_position_rad` | 0.0 | Calibrated open endpoint |
| `gripper_closed_position_rad` | 0.63 | Calibrated closed endpoint; reverse direction is supported |
| `gripper_impedance_kp` / `gripper_impedance_kd` | 12.5 / 0.05 | MD impedance spring and damping gains |
| `gripper_velocity_limit_rad_s` | 3.5 | Close target ramp speed/cap; not a measured speed guarantee |
| `gripper_torque_limit_nm` | 4.0 | Normal motor torque cap |
| `soft_close_slow_torque_limit_nm` | 4.0 | Close/hold torque cap, bounded by the normal cap |
| `close_target_lead_rad` | 0.04 | Maximum closing target lead at each control sample |
| `soft_close_closed_tol_rad` | 0.005 | Closed endpoint tolerance |
| `close_contact_torque_nm` | 0.35 | Minimum measured closing torque for inferred contact |
| `close_contact_velocity_rad_s` | 0.03 | Maximum absolute speed for contact/endpoint detection |
| `close_contact_duration_s` | 0.15 | Consecutive contact evidence required |
| `close_timeout_s` | 5.0 | Time allowed to reach contact or the endpoint |
| `close_feedback_timeout_s` | 0.25 | Maximum interval between valid control samples |
| `joint_state_publish_period_ms` | 50 | Batched feedback and close control period |
| `health_publish_period_ms` | 200 | Diagnostic status period |
| `md_can_response_timeout_100us` | 100 | CAN response window in 100 microsecond units |
| `md_host_timeout_ms` | 20 | Host USB wait deadline; must exceed the CAN window |
| `init_devices_zero` | false | Preserve existing zeros during initialization |

Legacy request fields `pre_close_enabled`, `pre_close_gap_mm`,
`pre_close_offset_mm`, and `fast_speed` are retained for wire compatibility
but ignored. `slow_speed` always selects the continuous target ramp:
`0.4 + 5.6 * slow_speed` rad/s, capped by `gripper_velocity_limit_rad_s`.
Values must be finite and in [0, 1]. Zero still means 0.4 rad/s.

Legacy launch parameters `soft_close_fast_torque_limit_nm`,
`soft_close_profile_acceleration_rad_s2`,
`soft_close_profile_deceleration_rad_s2`, and
`soft_close_fast_duration_ms` are accepted but unused.

## Three-drive example

For motors 111, 113, and 447, initialize without changing their zeros:

```bash
ros2 service call /md/init_devices candle_ros2/srv/InitDevices \
  "{device_ids: [111, 113, 447], mode: 'IMPEDANCE'}"
```

Calibration is deliberate: only at each finger's known mechanical open
reference, call:

```bash
ros2 service call /md/zero candle_ros2/srv/Generic \
  "{device_ids: [111, 113, 447]}"
```

Close one finger continuously at a 0.4 rad/s target ramp:

```bash
ros2 service call /md/soft_close_gripper candle_ros2/srv/SoftCloseGripper \
  "{device_ids: [111], slow_speed: 0.0}"
```

Open or disable:

```bash
ros2 service call /md/open_gripper candle_ros2/srv/Generic "{device_ids: [111]}"
ros2 service call /md/disable candle_ros2/srv/Generic "{device_ids: [111, 113, 447]}"
```

Opening retains the existing impedance move to the configured open target.
After disable, explicitly enable before sending another movement:

```bash
ros2 service call /md/enable candle_ros2/srv/Generic "{device_ids: [111]}"
```

The generic `md/close_gripper` uses the same bounded controller with the
node's `gripper_velocity_limit_rad_s` target ramp.

## Telemetry and position reference

`md/joint_states` publishes position, velocity and torque. Invalid or
recovering measurements are NaN. `md/health` publishes ordered arrays of
drive IDs, responsiveness, errors and detailed diagnostic strings.

Normal powered encoder readings already include accumulated turns. The driver
preserves their calibration offset, including across delayed samples.
The existing position recovery heuristic uses `encoder_wrap_period_rad`
(default 0.62831853), `position_recovery_max_delta_rad` (0.25),
`position_recovery_samples` (3), and `position_recovery_retry_ms` (25).

Recovery is only a nearest-angle estimate and cannot disambiguate large
unobserved movement or restore a lost reference after a node/drive restart.
It does not authorize movement or replay commands. If the reference is
uncertain, calibrate at the mechanical open stop before enabling.

## Tests

Hardware-free control and position regression tests:

```bash
colcon build --packages-select candle_ros2 --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select candle_ros2 --return-code-on-test-failure
colcon test-result --verbose
```

They cover immediate contact with a large object, lead bounds while holding,
free travel, signed contact debounce, reverse closing, delayed/stale feedback,
timeouts, invalid measurements and sparse powered encoder movement.

`ros2 run candle_ros2 test_grippers.sh` is a separate **hardware motion**
sequence for 111, 113, and 447. It requires confirmation, preserves zeros by
default, and exercises motors individually. Use `--zero` only at a verified
open reference. Its service acknowledgments alone do not verify a grasp;
inspect feedback and outcomes during commissioning.

## MAB documentation

- [Motion modes](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/MD/motion.html)
- [Safety and watchdog](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/MD/safety.html)
- [CANdleTool](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/CANdle-SDK/CANdleTool.html)
