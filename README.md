# CANdle ROS2

ROS 2 interfaces for MD motor controllers and PDS power distribution systems,
using the bundled [CANdle-SDK](https://github.com/mabrobotics/CANdle-SDK).
One process owns the bus; each added MD has independent command state.

## Impedance gripper control

Both `md/close_gripper` and `md/soft_close_gripper` now close in IMPEDANCE
mode from the first movement. There is no pre-close stage, gap estimate,
position-profile dependency, or timed fast-to-slow switch.

`md/open_gripper` uses a position-target ramp toward the known open endpoint
(0 rad in the MD20 profile). The target starts at measured position, advances
at 10 rad/s, and stays at most 0.14 rad ahead of measured position in the opening
direction. Opening uses Kp=30 and Kd=0.2, no integral, and a 2 Nm total torque
cap. These are the MD20 profile settings; the ramp limits target progression,
not measured motor speed, and the lead bound can slow that progression.
Normal opening never searches for a mechanical stop, uses the homing effort,
changes calibration, or zeros the encoder.

Normal opening has no position-acceptance tolerance or settled-endpoint success.
The independent `open_timeout_s` deadline (0.5 s) releases commanded torque,
Kp and Kd to zero regardless of final position. This remains a bounded opening
attempt, not confirmation of a fully open jaw. Release stays passive through
rebound or manual displacement until a new explicit command. Sustained opening
resistance with insufficient motion releases effort without a fault when within
`open_obstruction_tolerance_rad` (1 degree) of the calibrated open target. Outside
that band it still causes an obstruction stop. This band does not waive
stale/invalid feedback or hardware faults.
Closing endpoint recognition and calibration tolerances are unchanged.
Homing alone uses effort-qualified stop searches and disabled unloaded releases.

`gripper_travel_limits_enabled` controls the software absolute position guards.
It defaults to `true`; the MD20 profile sets it to `false`. When disabled, normal
opening, closing, holding, passive rebound and direct position commands do not
reject finite positions outside the learned/configured travel range. Calibration
still learns both stops and supplies the normal closed target. Opening and closing target
ramps, lead limits, homing consistency, torque/thermal/watchdog checks, position
reference validity and finite/float-representability checks remain active. Normal
opening retains its passive-release deadline; homing retains its search/release
deadlines. Numeric travel bounds remain available in diagnostics,
which also report `travel_limits_enabled`; homing reports this flag as `false`.
The parameter is loaded at startup; changing it requires restarting the node
and completing the usual calibration/readiness sequence.

The same normal-opening controller is used after homing and with a valid
manually established reference when homing is not required. The MD20 profile
still requires complete homing before accepting ordinary motion.

The 2026-10-01 restoration follows the normal-opening ramp in published baseline
`8665938ab5e5ccba3d0f94b70b4790526eab6d30`. It additionally releases PD after a
confirmed open endpoint; the baseline retained a position hold there. Historical
bench results below do not validate this restoration; physical revalidation is
pending.

Opening, closing, initialization and homing set and verify the firmware
`maxVelocity` ceiling at **1500 rad/s**, independently of the target ramps.
This separates the firmware trip from the target ramp. The MD20/GRIP profiles
retain a 0.04 rad closing lead and a 0.14 rad normal-opening lead, including
after learned calibration.
MAB documents no disabled value for this register; 1500 rad/s is its
[documented maximum](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/MD/config_calibration.html#limits-section),
not a commanded or guaranteed motor speed. The ROS driver does not save to flash.
Torque, thermal, watchdog and feedback guards still apply; software travel guards
apply when `gripper_travel_limits_enabled=true`.
Homing uses damped torque searches in both directions, with no position lead.

**Firmware v2.5.4 commissioning:** runtime register readback alone is insufficient.
Drive 345 still tripped near 10 rad/s with `maxVelocity=1500` read back; saving
that ceiling and rebooting removed the trip with the same driver and motion
settings. This is consistent with a boot-loaded internal limit. Provision the
saved ceiling once with the normal driver stopped and the drive disabled:
load its saved configuration by rebooting, back it up with `candletool md --id ID
config download /absolute/path/backup.cfg`, write `maxVelocity` to `1500`, then
save and verify after reboot. Saving itself restarts this firmware; wait for
communication to return. Compare configuration readbacks to ensure only the
velocity ceiling changed. Re-home afterward because reboot invalidates the
position reference. Do not save a running session's temporary homing gains or
motion targets. See [the six-drive validation](docs/validation/2026-09-18-velocity-ceiling.md).

Each closing motion starts with a target at the measured position. Each fresh feedback
sample advances it toward the requested endpoint at the configured
target ramp speed, bounded to `close_target_lead_rad` ahead of the measured
finger. The MD runs its own impedance PD controller with zero velocity
feedforward. Closing adds host-side integral torque (`gripper_impedance_ki`,
Nm/(rad*s)) using this bounded target error, not the remaining full sweep.
The MD torque cap applies to the total PD + integral output. Normal opening
uses PD only, with no integral. Integral increments are limited to one nominal control period and
`Ki * lead` Nm/s; conditional anti-windup prevents integration farther into
the torque cap or closing floor. Setting Ki to zero disables integration.

`gripper_closed_hold_torque_nm` is also the minimum closing-command torque
(0.25 Nm, clipped to both torque caps). On each fresh sample, if combined
PD + I would command less closing effort, the driver sends the floor as pure
torque with Kp/Kd zero; otherwise it keeps the original MD PD + I command,
without adding another bias. Thus damping cannot cancel the floor in that
command during normal closing. **Braking takes priority:** if measured closing
velocity exceeds the target ramp speed, or position is ahead of the ramp target
by more than the endpoint tolerance, native PD + I is restored without the
floor. A positive net-torque minimum cannot brake an unloaded jaw; applying it
unconditionally caused an overspeed/travel excursion on 111. The floor and
its anti-windup return when the jaw is back inside that envelope. This is not
a certified hard velocity limit. Normal opening retains PD braking. Native PD acts between
host samples, so this is not a guarantee about instantaneous measured torque
or jaw force. Revalidate physical motion after changing this setting.
The floor applies from closing startup through recovery (except braking) and
every closing hold. Setting it to zero disables the floor and settled-closed bias.
It does not apply to init/enable holds, opening, explicit disable or faults.

For a large object touching the fingers immediately, the position target
therefore cannot continue advancing through the remaining sweep. The lead
bound applies before contact is detected. At rest, the spring contribution
to motor torque is at most approximately `kp * lead`; the configured torque
cap also applies. These are motor torques, not calibrated fingertip forces.

Object contact is inferred from sustained closing torque and target error,
low signed average velocity, and a small position range over the contact
dwell. The range includes the preceding sample and rejects both continuous
travel and back-and-forth movement; individual noisy velocity samples no
longer reset the closing detector. Contact must be more than
`close_object_margin_rad` before the closed endpoint. Reaching an endpoint
does not establish object presence. After entering this near-closed margin,
object inference stays blocked until reopening farther than
`close_object_margin_rad + close_target_lead_rad` from the closed endpoint.
This hysteresis prevents small fingertip rebounds from repeatedly restarting
object torque buildup and the closing deadline. Normal opening instead uses
consecutive low measured and position-derived velocity for endpoint or
obstruction detection; normal opening has no settled-endpoint acceptance.

Once object contact is inferred, the driver clears the integral and switches
MD Kp/Kd to zero together with a feedforward command equal to the current
measured closing torque, bounded by the closing floor and torque caps. It then ramps torque at
`gripper_hold_torque_ramp_nm_s` toward `gripper_hold_torque_nm` (2 Nm), clipped
between the closing floor and both configured torque caps. This is total torque, not 2 Nm added to PD.
The ramp can finish after the 2-second closing deadline if contact was
confirmed before it. Ki affects approach/reacquisition, not final object
holding strength. Opening replaces the closing hold with its measured-position
initial PD command and bounded target ramp.

At a settled closed endpoint, `gripper_closed_hold_torque_nm` supplies a small
closing bias (0.25 Nm total) for thin foils and deformable fingertips. It also
applies when empty: position alone cannot identify a foil. The endpoint must
stay within `soft_close_closed_tol_rad` with low measured and position-derived
speed for `close_contact_duration_s`. The driver clears PD and I and commands
the bias/cap directly, without a zero-torque step or a ramp from below the
floor. Opening never applies this bias. Calibrated travel
sets its direction even when starting at or slightly beyond the endpoint.
When enabled, the configured travel guard also applies during this bias.

If neither an object nor a settled closed endpoint is confirmed within 2
seconds, capture the latest closing-direction torque, bounded by the closing
floor and caps (including zero/opposite measurements), clear I and PD, and
hold that value without boosting it to the object
setpoint. If this captured hold later settles at the endpoint, it transitions
to the closed bias. A settled closed bias remains active past the deadline;
it is not overwritten by another measured-torque capture.

During object torque buildup and holding, gradual inward compression updates
the anchor to the most-closed measured position without resetting the torque
ramp. Small outward movements do not move that anchor: reopening by more than
`close_target_lead_rad` triggers recovery. Closed-bias and unconfirmed deadline
holds retain their fixed anchor and recover on displacement in either direction.
For closing holds, speed above the selected closing-ramp rate (reported velocity
or sample-to-sample position speed) triggers recovery too. An inferred object
entering the near-closed margin also triggers recovery, before continuing the
object torque command; the near-closed inference block survives that recovery.
The bounded approach then reaches a settled endpoint or its existing timeout
torque capture, without interpreting small rebounds as fresh objects. Only a
settled endpoint selects exactly the closed bias instead of a potentially
higher hold; the floor does not change endpoint recognition or tolerance.
Recovery clears object evidence and old integral/holding torque, starts a new
target at the **measured position**, and resumes bounded closing with at least
the closing-command floor, not an intermediate zero-effort command.
Fresh contact evidence is required before torque can ramp
toward 2 Nm again. Each healthy reacquisition has the same 2-second deadline.
It never re-enables a faulted/disabled motor.

Recovery can temporarily reduce grip and will try to close again if the
object is removed. Keep fingers out and support the object against falling.
Slow object loss can resemble compression; position/torque inference alone
cannot reliably distinguish them. Torque caps and the endpoint guard still apply.
Invalid/stale feedback and hardware faults still disable. Calibrated-travel
violations disable when software travel enforcement is enabled. Neither sensorless contact inference nor these guards are
safety-rated, and motor torque is not calibrated jaw force.

Friction or a jam can resemble an object; tiny objects inside the endpoint
margin cannot be distinguished from empty closure with this detector. Gains,
the margin and thresholds require commissioning for each mechanism. Contact
torque must be lower than both torque caps and `kp * lead`.

### Timing and failures

Control reuses each drive's batched position/velocity/torque feedback. The
default timer is 50 ms. Delayed ticks advance the target by at most one
nominal period; they do not catch up with a large setpoint jump. Gaps above
`close_feedback_timeout_s`, invalid feedback, enabled calibrated-travel violations,
hardware faults and sustained opening obstruction abort the job and request
drive disable. Homing search/release deadline violations also fail. Only healthy
feedback can enter object, closed-bias or deadline torque hold; faults never do. Fault/disable/new
commands discard integral and captured-torque state.

Normal opening releases torque, Kp and Kd together at
its 0.5 s deadline, without disabling the motor. An opening timeout is not an
endpoint confirmation or proof of friction. The passive state remains monitored
for feedback, hardware faults and any enabled travel guards; it never retries
opening after rebound. Applications should verify low measured torque with
fresh feedback when unloaded release matters.

Contact and endpoint holds remain monitored until replaced or disabled.
Repeated closing requests replace the old command directly with a controller
starting at measured position, without a zero-torque cancellation. Normal
opening also replaces an already-enabled impedance command directly; other
states first require confirmed disable for setup. Normal passive release does
not toggle enable. Init/enable and
necessary mode changes confirm the actual MD
`state`/`motionModeStatus` readback; an SDK acknowledgement alone is insufficient.
Enable transitions are polled for up to 10 ms (individual transfers retain their
bus timeout), without repeating enable or clearing faults in the poll. Disable
first enters the drive's normal braking/quick-stop state; this is neither a
confirmed disable nor communication loss. Stops are requested once, cancel the
motion job immediately, and are checked asynchronously for up to 500 ms while
normal position telemetry and the other drives' control/watchdog traffic continue.
`/md/disable` returns success only after disabled-state readback, not just the
write acknowledgment. Timeout/fault returns failure and keeps motion inhibited;
only actual failed transfers trigger position-reference recovery. A pending stop
rejects re-enable, re-zero and configuration. Setup keeps a 10 ms fast path: if
braking needs longer, it stops asynchronously and the setup must be explicitly
retried after completion. Shutdown requests **all** disables before waiting for
their bounded confirmations. Neither the braking setting nor hardware watchdog
is changed. The
[MD state machine](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/MD/state_machine.html)
uses CiA402 state bits. Active jobs check enabled/impedance state in every batched
feedback frame, including torque holds and the enabled passive state after opening.
Homing releases instead require confirmed disabled state. Unexpected
disable/mode loss aborts and logs both registers rather than reaching a misleading
zero-torque closing timeout. An explicit enable/init is then required.

A setup failure never falls back to enabling a stale target. Configuration
writes that could change gains or torque limits are rejected while moving
or holding; cancel the operation first.

Explicit disable and fault stops cancel jobs and latch against automatic homing,
even if the hardware write fails or communication is lost again later. An
explicit home, enable or init is required to clear that latch; enable/init still
require a valid reference, and enable establishes a hold at measured position.
Communication recovery may schedule homing only when no such stop is latched.
Keep the MD hardware communication watchdog
enabled: software cannot confirm a disable across a disconnected bus.

Explicit disable, fault stops and successful recovery now confirm the disabled
state rather than trusting a write acknowledgment. Orderly node destruction
attempts disable/readback for every registered drive before releasing the bus;
a failed attempt does not prevent attempts on the remaining drives. Abrupt
process termination and a disconnected bus still require the hardware watchdog.

Open/close services return per-drive **command startup acknowledgments**, not final
grasp results. Logs report `contact inferred` with the torque ramp,
`object torque hold`, `closed endpoint`,
`timeout torque hold`, `open: timeout released` (endpoint unconfirmed),
or `recovering from measured position`. A torque-hold event is a controller
state, not proof that an object is securely retained.
Abort reasons are latched in `md/health` until explicit successful home/enable/init.
Applications must not treat service acceptance as proof of a successful pick.

## Build and run

This branch pins the audited CANdle-SDK transport changes as a submodule. Its
source history is published on `vendor/candle-sdk-transport-20261001` in this
same GitHub repository; that branch contains the SDK tree, not the ROS package.
The parent commit pins the exact SDK revision, so no patch files are needed.
After cloning, or when updating an existing checkout from the upstream SDK URL:

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

```bash
colcon build --packages-select candle_ros2
source install/setup.bash
ros2 launch candle_ros2 md_node_launch.py
```

PDS-only and combined launches remain `pds_node_launch.py` and
`both_launch.py`.

To build this checkout, including uncommitted changes, into the sibling GRIP
workspace's driver image:

```bash
cd ../grip
CANDLE_ROS2_SOURCE=../candle_ros2 docker compose \
  -f compose.yaml -f compose.local-candle.yaml build grip-candle-driver
```

The source must include initialized CANdle-SDK submodules. This builds
`grip-candle-gripper:local` without changing GRIP's published dependency pin or
starting hardware. The image records source hashes in
`/usr/local/share/grip/candle-local-source.sha256`; `GRIP_CANDLE_LOCAL_IMAGE`
selects a different tag. Starting it requires the same Compose override and
robot-selection environment and physically homes the selected grippers.

The node accepts `bus:=USB|SPI`, `data_rate:=1M|2M|5M|8M`,
`usb_serial:=SERIAL` (empty selects the first available adapter), and
`default_qos:=Reliable|BestEffort`. The MD launch exposes the settings below.

## Configuration

| Parameter | Default | Meaning |
| --- | --- | --- |
| `gripper_open_position_rad` | 0.0 | Calibrated open endpoint |
| `gripper_closed_position_rad` | 0.75 | Closing direction and fallback closed target; successful homing replaces the target with each drive's measured loaded closed stop; reverse direction is supported |
| `gripper_impedance_kp` / `gripper_impedance_kd` | 12.5 / 0.05 | MD impedance spring and damping gains |
| `gripper_impedance_ki` | 2.0 | Host integral gain in Nm/(rad*s), closing only; zero disables it |
| `gripper_hold_torque_nm` | 2.0 | Desired inferred-object torque, clipped to both torque caps |
| `gripper_closed_hold_torque_nm` | 0.25 | Minimum closing-command torque and settled-closed hold (including empty), clipped to both caps; zero removes floor/bias; never applied to opening or faults |
| `gripper_hold_torque_ramp_nm_s` | 2.0 | Maximum object-hold torque slew, Nm/s; closing floor applies immediately and settled endpoint selects the closed bias directly |
| `close_object_margin_rad` | 0.01 | Exclude near-closed contact from object inference; must be at least endpoint tolerance |
| `gripper_velocity_limit_rad_s` | 1.5 | Closing target ramp speed/cap in rad/s |
| `gripper_open_velocity_limit_rad_s` | 10.0 | Normal-opening target ramp toward the known open endpoint; independent of the firmware velocity ceiling |
| `gripper_torque_limit_nm` | 2.0 | Normal motor torque cap |
| `soft_close_slow_torque_limit_nm` | 2.0 | Open/close/hold total torque cap, bounded by the normal cap |
| `close_target_lead_rad` | 0.04 | Maximum closing target lead; also displacement that starts hold reacquisition |
| `open_target_lead_rad` | closing lead | Maximum normal-opening target lead ahead of measured position; MD20 profile uses 0.14 rad |
| `soft_close_closed_tol_rad` | 0.005 | Closing endpoint tolerance, zero-command readback and local homing settling range; not normal-opening acceptance |
| `gripper_travel_limits_enabled` | true | Enforce software absolute travel guards for measured positions and targets; MD20 profile sets false; finite/reference checks and calibrated closed targets remain active |
| `gripper_travel_min_rad` / `gripper_travel_max_rad` | -0.05 / 0.80 | Fallback normal-controller guards; calibration learns the closed bound plus margin and extends the open bound only if needed to contain the measured powered stop plus tolerance |
| `homing_travel_margin_rad` | 0.5 | Nonnegative extra normal travel beyond each drive's learned closed stop; applied after calibration |
| `homing_torque_nm` / `homing_damping_nm_s_rad` | 1.0 / 0.5 | Homing-only search effort and damping; stop dwell needs measured directional effort at least half this torque |
| `homing_torque_limit_nm` | 2.0 | Total search torque cap, including damping |
| `homing_contact_duration_s` | 0.25 | Powered-stop and unloaded-release position dwell |
| `homing_timeout_s` | 2.0 | Deadline for each powered homing search; each disabled homing release has its own fixed 3 s deadline |
| `close_contact_torque_nm` | 0.35 | Minimum directional torque for closing contact or normal-opening obstruction |
| `close_contact_velocity_rad_s` | 0.03 | Object-window average velocity/range threshold; instantaneous/position-derived threshold for normal opening and closed-endpoint settling |
| `close_contact_duration_s` | 0.15 | Consecutive contact or settled-endpoint evidence required |
| `close_timeout_s` | 2.0 | No-object closing/reacquisition torque-capture deadline |
| `open_obstruction_tolerance_rad` | 0.017453292519943295 | Opening resistance inside this 1-degree band releases effort without a jam fault; closing is unchanged |
| `open_timeout_s` | 0.5 | Normal-opening passive-release deadline, including after calibration; timeout leaves the endpoint unconfirmed |
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

### Independent lead tuning (111, 2026-09-16)

`config/md20_gripper.yaml` preserves the measured MD20 tuning separately from
the generic launch defaults. It selects closing/opening ramps of 2/10 rad/s
and target leads of 0.04/0.14 rad, with Kp 30, Kd 0.2, closing Ki 5 and a
100 Hz control period. The 2 Nm cap/object hold and 0.25 Nm closed-bias setting
are unchanged. The profile retains 0.0/0.75 rad configured targets and
-0.05/0.80 rad fallback guards. Successful homing now learns each drive's
closed target and replaces its closed-side guard as described below.
The closing lead still also controls integral slew and hold-recovery displacement;
the opening lead does not change those closing behaviors.
The September 17 closing-command floor uses the same 0.25 Nm setting, but
changes command shaping; the historical speeds below predate that change.

Load the profile using native ROS parameter-file support (with no other process
owning the same adapter):

```bash
ros2 run candle_ros2 candle_container --ros-args --params-file \
  "$(ros2 pkg prefix candle_ros2)/share/candle_ros2/config/md20_gripper.yaml"
```

Loading this profile does not add, initialize, enable or zero any motor. Parameters
are read at startup; changing them on `candle_config` at runtime does not retune
an existing controller. The measurements were made on unobstructed **111 only**,
not on 113/447 or with an object. These are target ramp rates, not hard limits on
instantaneous measured velocity. Keep the physical area clear when testing.

Five cycles at the final settings (including one loaded from the installed YAML)
gave the following position-derived speeds over the same 0.10–0.60 rad segment:

| Motion | Target lead (rad) | Target ramp (rad/s) | Median measured speed (rad/s) | Measured range (rad/s) |
| --- | --- | --- | --- | --- |
| Closing | 0.04 | 2.0 | 1.97 | 1.95–1.98 |
| Opening | 0.14 | 10.0 | 9.91 | 9.84–10.06 |

Speeds use interpolated position crossings from 100 Hz telemetry, not whole-stroke
averages; opening crosses this segment in about five samples. Instantaneous
reported velocity peaked at 2.82/12.29 rad/s (closing/opening). Peak sampled
absolute torque was 1.37 Nm; positions stayed between -0.01588 and 0.74698 rad.
Released-position means were within 0.00006 rad of zero without re-zeroing.
Four openings used the existing 0 Nm timeout release; one confirmed the settled
open endpoint. This does not claim every endpoint settled before timeout.
All five cycles completed without drive faults and ended disabled; 113/447 were
not driven. Loaded grasps and the other grippers still need separate validation.
The full sweep comprised nine completed hardware cycles plus one preflight-only
abort before the devices had been registered (no motion); peak motor temperature
was 34.16 C. The [compact results](docs/validation/2026-09-16-speed-111.csv) are
versioned; raw CSV/JSON remain under `/tmp/candle-closed-bias.tGtPAk/` on the tuning host.

### Initial tuning measurements (111, 2026-09-15)

After operator-confirmed open calibration of **111 only**, one empty close/open
cycle per setting used endpoints 0.0/0.7 rad, Kd 0.2, a 0.4 rad/s target ramp,
1 Nm torque caps, a 5 s timeout and the unchanged 0.005 rad endpoint tolerance.
Positions below are means over seconds 4–5, before timeout disabled the drive:

| Kp | Lead (rad) | Closing position (rad) | Opening position (rad) | Peak sampled absolute torque (Nm) |
| --- | --- | --- | --- | --- |
| 20 | 0.040 | 0.6913 | 0.0108 | 0.225 |
| 25 | 0.032 | 0.6925 | 0.0081 | 0.209 |

Both directions still timed out at both settings; these are **not commissioned
gains or successful endpoint results**. No contact was reported during the free
sweep. The 1 Nm cap was not reached. Higher stiffness reduced stationary error
but did not eliminate it. 113 and 447 remained disabled and were not re-zeroed.
Earlier readings of 111 use its previous zero and are not directly comparable.

### Empty Ki trial (2026-09-15, drive 111)

After the later operator-confirmed open re-zero, the empty trial used Kp 30,
Kd 0.2, Ki 2 Nm/(rad*s), a 1 rad/s ramp, 10 ms control period, 0.026667 rad
lead, 1 Nm total torque cap, and a 2 s timeout. These runtime overrides differ
from some general launch defaults; no zero was changed for this trial.

Closing captured 0.1299 Nm at 0.6967 rad after 2.001 s. During the subsequent
five-second torque hold, measured position stayed within 0.69647–0.69685 rad
and mean measured torque was 0.1298 Nm. Peak sampled closing torque was
0.2352 Nm. Opening restored PD but timed out at 0.0072 rad (outside the
0.005 rad tolerance); peak sampled opening torque was 0.2672 Nm. After
disable, the finger rebounded to 0.0263 rad. All three drives ended disabled;
113 and 447 were not moved. This validates one empty torque-hold transition,
not object retention or a commissioned grasp force.

### Direct-opening trial and ramp restoration (2026-09-15, drive 111)

With closing raised to 1.2 rad/s, closing held at 0.6963 rad and 0.1401 Nm.
A direct open-endpoint step then reached a sampled 24.3 rad/s and overshot
to -0.0877 rad, triggering disable. The final disabled reading was -0.0447 rad.
No retry or re-zero was performed. This direct-opening configuration failed
the travel check and has been removed. Opening now uses a lead-limited ramp
with its own 10 rad/s setting, and closing is set to 1.5 rad/s. A subsequent
bounded trial closed to 0.6963 rad and opened to 0.0044 rad without crossing
zero; opening still failed the settling dwell before its 2-second deadline.

### Explicit object hold and reacquisition

The 2 Nm object-hold ramp and movement-triggered reacquisition passed initial
hardware-free regression tests. Replaying the earlier bottle trace inferred
contact at 0.441 s; replaying the empty trace did not infer an object. Replay
validates contact evidence only, not the mechanism's response to new commands.

The first bottle trial (111, 2026-09-15) exposed a recovery loop: inward
compression repeatedly exceeded the fixed 0.026667 rad hold-anchor window.
The recorded 6.681 s interval contained ten recoveries and a peak measured
torque of 1.7146 Nm, without establishing a stable 2 Nm hold. The trial was
stopped with explicit disable; zeros were unchanged.

The correction follows gradual inward compression during torque buildup and
holding, but retains recovery on accumulated reopening, fast movement, or
entry into the closed-end margin. The correction passed 35 hardware-free tests,
including a regression that reproduced the reset before the fix. The next bottle
trial (111, 2026-09-16) reached a 2 Nm object hold with no recovery restarts in
the recorded 6.42 s. Peak sampled torque was 2.0276 Nm; the last two seconds
averaged 2.0000 Nm. Position continued inward from contact at 0.4530 rad to
0.5915 rad at the end of recording and 0.6301 rad at the subsequent check.
The operator reported a good grasp. These are motor torque estimates, not jaw
force or a proof of general object retention. Zeros were unchanged.

Continuing that 2 Nm hold for roughly ten minutes triggered a motor-temperature
fault (`hardware: Error Motor Temperature`); the driver disabled 111 and latched
the fault. The short successful grasp does not validate continuous holding at
2 Nm, let alone a higher torque. Cooling and thermal/duty-cycle assessment are
required before commissioning the higher setting; do not bypass the fault.

Subsequent supervised bottle trials (111, 2026-09-16) used the unchanged 90 C
motor cutoff. At 2 Nm, starting at 39.54 C, holding lasted about 10 min 44 s;
at 2.5 Nm, starting at 40.11 C, it lasted about 3 min 2 s. Both recorded
`Error Motor Temperature` in the next 200 ms health sample after the monitor
stopped on movement recovery. These are single-run measurements, not continuous
duty ratings. All motors were left disabled; zeros and thermal limits were unchanged.
The selected object-hold target and both torque caps are now 2 Nm. The 0.25 Nm
closed bias remains configured, but foil retention still needs separate testing.

The subsequent three-gripper cycle stopped during empty 113's close: contact
near 0.687 rad alternated with near-closed recovery around 0.690 rad, restarting
the deadline nine times. Near-closed inference now uses the existing lead as
hysteresis. Replaying the 401 recorded 113 samples reduces this to one recovery
and a 2-second captured-torque hold; 111's bottle-hold replay is unchanged.
Replay validates decisions on old feedback, not the mechanism's response to
new commands. Subsequent physical cycles completed under the opening-timeout
release policy; the later missed close on 113 is described in the Tests section.

On 2026-09-16, only 113 was re-zeroed at the operator-selected open position
(previously about -0.0076 rad). Earlier 113 positions use the old reference.
The travel margin was first widened to 0.01 rad, then to 0.05 rad at the
operator's request. At that point the commanded endpoints remained 0.0 and 0.7 rad, with an
unchanged 0.005 rad settling tolerance. This does not resolve the later
447 opening-resistance stop or 111's closed-bias overshoot; at that earlier point,
the randomized-bottle test had not completed a full cycle. Subsequent ten-cycle
testing completed with one missed close on 113, rather than validating every grasp.

All three were subsequently re-zeroed at their operator-selected open positions.
Manual full-close readings were 0.7362 rad (111), 0.7100 rad (113), and 0.7166 rad
(447). The shared closed target is now 0.75 rad, with open still 0.0 rad and the
settled-closed bias still 0.25 Nm. Absolute travel parameters replace
`gripper_travel_tolerance_rad` to retain the requested -0.05/0.75 rad limits when
targets change. Object holding and both torque caps remain 2 Nm. A motor that
stalls more than 0.01 rad before the new closed target can still be classified
as object contact; the measured manual endpoints alone do not guarantee the
0.25 Nm closed-bias state. No physical cycle has yet validated this new target.

## Three-drive example

With the MD20 profile, empty the jaws, clear the opening and closing paths,
add the drives, and home them one at a time before normal initialization.
Homing replies only after the full open-close-open calibration completes:

```bash
ros2 service call /md/add_mds candle_ros2/srv/AddDevices "{device_ids: [113, 447, 111]}"
ros2 service call /md/home candle_ros2/srv/Generic "{device_ids: [113]}"
ros2 service call /md/home candle_ros2/srv/Generic "{device_ids: [447]}"
ros2 service call /md/home candle_ros2/srv/Generic "{device_ids: [111]}"
ros2 service call /md/init_devices candle_ros2/srv/InitDevices \
  "{device_ids: [111, 113, 447], mode: 'IMPEDANCE'}"
```

Manual calibration remains available only at a verified mechanical open
reference. It is not a substitute for the MD20 profile's homing requirement:

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

After homing, normal opening ramps toward the known open endpoint with PD only.
A settled endpoint releases all effort; a healthy 0.5 s timeout also releases,
but reports the endpoint as unconfirmed. Neither outcome changes calibration.
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
It also publishes `motor_temperature_c`, `mosfet_temperature_c`, and the
read-only configured `motor_shutdown_temperature_c` in the same drive order.
Unavailable/non-finite temperature reads are NaN, mark health as erroneous,
and abort an active gripper job. A zero motor temperature may indicate a
missing thermistor; it must not be treated as a valid thermal-test reading.
These fields extend the ROS message: rebuild consumers against the new type.
Health reporting does not change temperature limits or clear hardware faults.

`md/zero` first disables and verifies the drive, writes zero, and checks fresh
encoder readback before resetting the software reference. It invalidates any
learned travel calibration and leaves the drive disabled. A successful response
confirms zeroing only; explicit homing is required again for a calibrated gripper.
An acknowledgement or readback failure leaves the reference faulted and motion
inhibited. Zeroing does not establish the physical mechanical reference by itself.

Normal powered encoder readings already include accumulated turns. The driver
preserves their calibration offset, including across delayed samples.
With `homing_required=false`, the legacy position recovery heuristic uses `encoder_wrap_period_rad`
(default 0.62831853), `position_recovery_max_delta_rad` (0.25),
`position_recovery_samples` (3), and `position_recovery_retry_ms` (25).

Recovery is only a nearest-angle estimate and cannot disambiguate large
unobserved movement or restore a lost reference after a node/drive restart.
It does not authorize movement or replay commands. If the reference is
uncertain, calibrate at the mechanical open stop before enabling.

### Mechanical-stop homing (MD20 profile)

`homing_required=true` blocks normal motion until `/md/home` succeeds. The
existing service performs the complete open-close-open calibration; there is no
separate calibration service. Its deferred success response is sent only after
all three searches, releases, and final reference verification complete. GRIP calls
it sequentially on every launch. The service accepts exactly one ID; concurrent
searches and configuration changes during a search are rejected. On a returning
drive after communication/power loss, the driver schedules one homing attempt,
one drive at a time in discovery order. It never resumes a previous grasp.
An explicit disable or fault stop cancels pending automatic homing and keeps it
blocked across subsequent communication losses. A search failure leaves the
reference invalid and requires an explicit retry, not an endless retry loop.

Homing requires **empty jaws and unobstructed opening and closing paths**.
An explicit home also requires `gripper_open_position_rad=0.0` and
`init_devices_zero=false`, so later initialization cannot replace the learned reference.
Every search uses 1 Nm in its search direction, Kp=0, Kd=0.5 Nm/(rad/s), zero target
velocity, and a 2 Nm cap. There is no position spring, integral, or stop at a
previous target. The sequence is:

1. Search toward the open stop (-1 Nm on these grippers). Require a position
   range <=0.005 rad and measured directional torque >=0.5 Nm for 0.25 s.
   Movement or effort below half the requested seek torque resets the dwell;
   stationary idle feedback cannot establish a stop. Clear PD/torque and
   disable, allow rebound, then require a fresh 0.25 s position window <=0.005 rad
   with torque magnitude <=0.1 Nm. Set and verify the open zero while disabled.
2. Search toward the closed stop with the same effort and settling checks.
   Record this drive's loaded closed endpoint in the new open frame, then clear
   PD/torque, disable, and wait for unloaded settling. Rebound does not replace
   the recorded loaded endpoint.
3. Search toward the open stop again and record its powered position in the
   established zero frame (`openPosition`). Clear PD/torque, disable, and wait for
   unloaded settling. Accept a settled offset from the original open zero without
   re-zeroing; retain and report the measured position. Restore normal limits and
   enter the normal released-ready state at 0 Nm after settling completes.

Successful homing uses that drive's learned loaded endpoint as its normal closed
target. For positive-direction closure, its normal maximum position is
`learned_closed_endpoint + homing_travel_margin_rad`. Its minimum is
`min(configured_minimum, learned_powered_open - endpoint_tolerance)` to contain
the measured loaded stop. For negative-direction closure, the minimum
becomes `learned_closed_endpoint - homing_travel_margin_rad` and the maximum is
`max(configured_maximum, learned_powered_open + endpoint_tolerance)`. The default
closed-side margin is **0.5 rad**; the open-side allowance is only the unchanged
endpoint tolerance. Homing has no preset position or travel bounds;
these normal-motion bounds are applied after complete calibration only when
`gripper_travel_limits_enabled=true`.
Calibration is held per drive in memory and is invalidated by zeroing, reference
loss, or starting another home. A partial or failed home never installs it.

`homing_torque_nm=1.0`, `homing_damping_nm_s_rad=0.5`, `homing_torque_limit_nm=2.0`,
`homing_contact_duration_s=0.25` and `homing_timeout_s=2.0` configure the stop
dwell and each powered-search deadline. Each disabled release has a separate
3 s deadline (`ImpedanceHome::Config::releaseTimeout`, not a ROS parameter).
Searches and disabled releases have no preset position or travel bound, allowing
the sequence to measure the actual stops. Service callers must allow **three
search deadlines plus three release deadlines, plus setup and communication
time** per drive: 15 s plus overhead with the defaults. GRIP allows at least
30 s per home service call. Startup callers using the former single-search
budget must increase it.
The firmware cap bounds the combined feedforward/damping demand during homing.
These search, effort-dwell and disabled-release rules apply only to calibration.
Normal opening targets the known open position with its independent 10 rad/s
PD ramp, 0.14 rad lead and 0.5 s passive-release deadline. It never reuses a stop
search or changes the learned calibration. Closing keeps its separate 2 s
torque-capture deadline. Software travel guards apply to normal motion only
when `gripper_travel_limits_enabled=true`; the MD20 profile keeps them disabled.

There is no measured-speed homing abort or velocity-based settling criterion.
The firmware velocity ceiling is 1500 rad/s, independently of the normal-motion
target ramps. The torque cap and velocity ceiling are read back before enable.
Reverse travel, stale/nonfinite feedback or
unhealthy hardware still aborts. Measured torque above the cap plus 0.1 Nm also
aborts; a configured cap is not a guarantee that reported torque cannot overshoot.
A nonzero hardware watchdog of at most 250 ms is required. Existing restrictive
firmware position limits are rejected rather than silently removed.

A homing timeout is never success. The learned guards are
software checks; firmware position limits and flash contents are unchanged.
The former `homing_max_travel_rad`, `homing_velocity_limit_rad_s`, `homing_release_max_travel_rad`,
`homing_ramp_speed_rad_s`, `homing_target_lead_rad` and `homing_contact_torque_nm`
parameters are removed.

**Startup/power restoration can open, close, and reopen the grippers: keep
the jaws empty and both paths clear.** Sensorless homing assumes repeatable
mechanical stops. A closed-stop search can mistake an object or jam for the
mechanical endpoint; a successful sensorless sequence does not prove the closing
path was unobstructed. The torque settings are control limits, not guaranteed
collision-force bounds. Homing torque searches have no commanded velocity ramp;
normal opening has the separate target ramp described above.

## Tests

Core tests cover effort-qualified homing, independent loaded stop calibration,
homing search/release deadlines and accepted released offsets. Normal-opening
regressions cover the independent ramp and lead, position-independent deadline
release, unchanged calibration, obstruction and invalid/stale feedback. Command-safety tests cover persistent disable recovery
gates and invalid register values. These are offline checks; they do not establish
physical endpoint repeatability or grip retention.

Hardware-free control and position regression tests:

```bash
colcon build --packages-select candle_ros2 --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select candle_ros2 --return-code-on-test-failure
colcon test-result --verbose
```

They cover immediate contact with a large object, lead bounds while holding,
free travel, independently ramped opening to zero, signed contact debounce, position progress despite
zero reported velocity, endpoint settling dwell, reverse closing, delayed/stale
feedback, timeouts, invalid measurements and sparse powered encoder movement.
Integral tests cover bounded error/time increments, signed anti-windup,
closing torque capture without double-counting PD, opening with no integral,
zero-torque opening-timeout release with no rebound reacquisition and retained fault checks,
bounded recovery in either direction, noisy contact evidence, explicit torque
ramps through gradual compression, accumulated reopening and fast-position
recovery, endpoint exclusions, capped closing floors and settling after
timeout, calibrated bias direction, and faults taking precedence over recovery.
Floor tests cover combined PD + I (including damping and reversed error), both
directions, a lower hardware cap, zero-floor opt-out, startup/recovery/timeout/
endpoint handoffs, unchanged above-floor effort, lower-bound anti-windup and
zero command after failure. Opening release remains zero torque with PD off.
Drive-state tests cover delayed and refused transitions, wrong motion mode,
fault/quick-stop state bits, transfer errors, and no automatic enable retries.

In the 2026-09-16 ten-cycle trial, cycle 8's gripper 113 stayed at approximately
0.0259 rad with torque-sensor noise only, yet reached the controller's normal
deadline hold. The old recorder counted that as completion. The recorder now
rejects a close with neither position change above 0.005 rad, a closed endpoint,
nor holding torque of at least 0.1 Nm; a large object at the starting position
still passes when meaningful holding torque is measured. The old recording did
not include hardware state/mode readback, so it cannot prove that a lost enable
transition caused this particular event. After the startup/monitoring changes,
one supervised 113 retest and ten further cycles completed with no missed close
or state fault, using the previous 1.5 rad/s closing ramp and 0.0267 rad shared
lead. This does not prove the historical cause or validate 113 at the new speeds.

### Guarded commissioning recorder

`record_gripper_cycles.py` is the recorder used for these checks, now installed
with the package. Its hardware-free guard self-test also runs under CTest:

```bash
python3 scripts/record_gripper_cycles.py --self-test
```

For physical runs, use the MD20 profile, register the selected IDs with
`md/add_mds`, and confirm they are disabled before starting. With the area
checked clear by the operator:

```bash
ros2 run candle_ros2 record_gripper_cycles.py \
  --ids 113 --cycles 10 --run /path/to/new_trial.csv --clear
```

For a standalone Docker bench server, run the installed `candle_container`
executable directly as PID 1 (after sourcing ROS and the workspace). The
`ros2 run` wrapper did not forward Docker's SIGINT in the shutdown check;
direct execution allowed the driver's disable/readback cleanup to run.

This script targets the unnamespaced bench `/md` and `/candle_config` interfaces,
not the production `/grip/candle` namespace. It checks the calibrated geometry,
torque caps, timeout and zero-preservation settings before init; records loaded
gains/ramps/leads; guards fresh feedback/health/temperature; rejects a no-response
close; and distinguishes opening endpoint confirmation from timeout release.
It closes selected IDs sequentially, waits 10 seconds, opens them together, then
disables and verifies healthy released feedback. Every cycle has exclusive-create
CSV/JSON files. A fault, rejected command or unconfirmed disable stops the batch
with a nonzero exit status; it never zeros, changes limits or retries a fault.
For supervised disturbance observation, `--hold-seconds 30` extends that wait
without changing the controller. The accepted range is 1–30 seconds; opening is
automatic at its end. Support the object and keep hands outside the finger sweep.

The main GRIP integration is a separate repository change. Its prepared startup
validates the loaded parameters and homes each drive; its dependency pin must be
updated only after this branch is pushed. See the [release-check record](docs/validation/2026-09-16-release-check.md)
for what is proven and what still needs hardware validation.

`ros2 run candle_ros2 test_grippers.sh` is a separate **hardware motion**
sequence for 111, 113, and 447. It requires confirmation, preserves zeros by
default, and exercises motors individually. Use `--zero` only at a verified
open reference. Its service acknowledgments alone do not verify a grasp;
inspect feedback and outcomes during commissioning.

## Fault diagnostics

The driver emits `CANDLE_DIAG { ... }` JSON records through its existing ROS
logger. Preserve the container/ROS logs to retain them; no separate recorder,
topic, register write, error clearing, or automatic recovery is introduced.
`schema` is currently `1`. `drive_id`, ROS nanoseconds, and monotonic nanoseconds
identify the drive and time. `fault_sequence` is scoped to one node process.

- `drive_fault` captures bounded memory **before** the existing stop request
  erases the controller job or disables the drive. Logging occurs in the next
  health callback, with separate capture and emission times. A normal node
  shutdown also flushes pending faults after its existing disable sequence,
  without any additional diagnostic register reads. Explicit `/disable`
  requests do not create or rate-limit a fault record. `at_stop` is the
  latest feedback sample, including its validity and age, raw/logical position,
  velocity, torque, state/mode, last attempted impedance command and write
  result, gains, command timestamp, learned travel, torque/feedback/time limits,
  and measured feedback interval. Impedance command target velocity is zero;
  the independent firmware/profile velocity limits appear in register records.
  The sample's controller fields describe the state before that feedback was
  processed; `controller_state_at_capture`, `phase_at_capture`, and the
  elapsed/settled fields at capture describe the resulting fault state.
- `fault_history` records the preceding 64 samples at **at most 20 Hz**, in
  chronological chunks of eight. The exact most recent tick is retained in
  `at_stop` even when it fell between history samples. Match chunks by drive,
  sequence and `offset`. Repeated fault captures are limited to one per drive
  per second and one pending record; `suppressed_faults` reports coalesced calls.
- `register_snapshot` reads one of six groups listed below. Communication
  status is read **even when quick status is zero**;
  `communication_can_watchdog_warning` decodes communication bit 30.
  `miscStatus` (`0x813`) is never queried: a disabled-drive single-register
  probe timed out on the deployed firmware. Its JSON value is `null`, with
  `misc_status_queried: false`.
- `diagnostic_polling_disabled` reports the first optional read failure. The
  entire node permanently stops optional diagnostic polling until restart;
  cached feedback history and fault logging continue. There are no retries,
  including post-fault refreshes.

| Group | Registers | MD request bytes | Full USB OUT bytes |
|---|---|---:|---:|
| `status_core` | Quick, main/aux encoder, calibration, bridge status | 30 | 35 |
| `status_protection` | Hardware, communication, motion, homing status | 26 | 31 |
| `command` | State, mode, CAN watchdog, Kp, Kd, max torque | 31 | 36 |
| `targets` | Position/velocity/torque targets, max velocity, profile velocity | 32 | 37 |
| `limits_supply` | Position min/max, DC bus voltage | 20 | 25 |
| `thermal` | Motor/MOSFET temperature, motor shutdown temperature | 17 | 22 |

MD sizes include the two-byte MD header and each register address. USB sizes
also include CANdle's five-byte synchronous request envelope. These exact
six groups completed 54 disabled-drive reads across all six deployed drives
on 2026-09-30, with a maximum observed duration below 1 ms. This is a read-only
compatibility check, not proof of timing under motion.

Every register group has its own start/end timestamps, duration, `read_ok` and
`read_error`. Use `read_ok` as the success flag: the SDK enum encodes `OK` as
`1`, not zero. Failed reads have `values: null`; non-finite numbers are JSON
`null`. Groups are separate transactions, **not an atomic drive snapshot**.
Fault records embed the previously cached groups with their original times;
calculate their age instead of treating them as measurements at the fault.
Priority follow-up groups carry the fault sequence and
`after_fault_stop_request: true`: they show registers after a disable was
requested, potentially after its mode/command changes, not the pre-stop state.
`disable_pending` distinguishes an outstanding disable transition.

Diagnostics reuse existing control feedback without additional reads. Extra
register traffic is limited globally to **one read-only batch of at most
32 MD bytes / 37 full USB OUT bytes per health callback, no more often than
every 200 ms**. Six groups rotate through the drives (nominally 7.2 seconds
per complete sweep for six drives at 200 ms); pending faults take priority.
No optional reads occur while any drive is homing or any disable transition
is pending. When homing is required, the complete initial calibration of all
known drives must finish before polling starts, including the gaps between
sequential homings. This startup readiness remains latched after a later
fault so post-stop registers can be inspected once disable completes.
`homing_required=false` bypasses only the initial-calibration requirement.

Reads use local register objects, never overwrite the control cache, and run
in the existing mutually exclusive callback group. A read failure opens the
optional-polling circuit permanently for that node process and makes no new
control or health decision. In-memory fault logging runs even when polling
is gated or disabled. The first diagnostic read after a fault occurs only
after the existing stop path and every pending disable have completed.

Integer state codes follow the controller enums:

| Field | Values |
|---|---|
| `operation` | 0 idle/no supervised job; 1 homing; 2 reserved (retired calibrated-open controller, historical traces only); 3 close; 4 normal open |
| Homing `phase` | 0 opening; 1 closing; 2 reopening; 3 ready; 4 failed |
| Homing `control_state` | 0 searching; 1 found; 2 releasing; 3 ready; 4 failed |
| Close/normal-open `phase` and `control_state` | 0 approaching (enum `Closing`); 1 contact; 2 endpoint reached (enum `Closed`); 3 torque hold; 4 failed. For normal opening, states 2 and 3 both release all effort; state 3 leaves the endpoint unconfirmed. |

`-1` denotes an unavailable phase/state. These diagnostics establish software
commands and measured drive behavior; they do not establish physical jaw gap,
object retention, or the internal firmware cause of a fault.

## MAB documentation

- [Motion modes](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/MD/motion.html)
- [Safety and watchdog](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/MD/safety.html)
- [CANdleTool](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/CANdle-SDK/CANdleTool.html)

### Startup reboot before discovery

`/md/startup_reboot` (`Generic`) accepts an explicit unique list of drive IDs, once
per process and only before any MD is registered. It logs state and fault registers,
confirms all selected drives disabled or in latched FAULT, reboots each once, and
requires disabled/idle, zero quick status and target torque, and a 1–250 ms watchdog.
There is no enable, zero, flash-save or homing in this service. The caller must add
and home drives afterward. Missing communication, a persistent fault or a failed
reboot acknowledgment prevents success; the service never loops resets.
