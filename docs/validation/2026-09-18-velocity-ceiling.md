# Six-drive velocity ceiling validation — 2026-09-18

## Change

The gripper configuration path now writes and reads back `maxVelocity=1500`
independently of `profileVelocity`. Initialization, homing, closing and opening
all use this path. This is the manufacturer's documented register ceiling,
not a speed command or a literal disabled value. The driver does not save flash.
The 0.14 rad opening lead, 0.04 rad closing lead, 10/2 rad/s target ramps,
2 Nm motion torque cap, watchdogs, thermal settings and feedback/travel guards
are unchanged. Homing remains a damped torque search with no position lead.

Source changes are in this repository, without GRIP patch files. The candidate
was built in the GRIP ROS Jazzy image, with USB absent during build and tests.
All four CTest entries passed (colcon reports 63 tests, zero failures), including readback failure, clamped/nonfinite
limits and phase-independent ceiling regression coverage.

Tested binary SHA-256:
`ec3f376d4e3f0785ee6aa32c5717f53e1743a5182c9039f7610c225c243a3b03`.
Local candidate image:
`grip-candle-gripper:velocity-ceiling-20260918`, also tagged
`grip-candle-gripper:jazzy` for the physical run. No dependency revision was
published or pinned; a clean GRIP rebuild still requires integrating this source.

## Firmware finding

All six drives run v2.5.4.r, commit `5938df37`.
The first run homed and closed all six successfully. Five opened without a
velocity fault, with sampled peaks of 11.65–12.41 rad/s. Drive 345 still faulted
approximately 32 ms after opening began, at position 0.605 rad. Its last sampled
peak speed was 10.454 rad/s. Motion status was `0x2000002` (torque warning plus
velocity limit error), state 8, mode 0. Direct hardware inspection after stopping
the driver confirmed `maxVelocity=1500`, so runtime register readback was not
enough to eliminate this fault.

After reboot, 345's saved ceiling read 10 rad/s. With the drive disabled and
normal driver stopped, its boot configuration was backed up, only `maxVelocity`
was changed to 1500, and the configuration was saved. Saving restarts this
firmware; an additional reboot verified persistence. Before/after downloaded
configurations differ only in `max velocity`. Watchdog remained 100 ms and motor
thermal shutdown remained 90 C.

The second full home/close/open cycle, using the identical candidate binary and
motion settings, had no drive errors. Drive 345 opened at a sampled peak of
11.39 rad/s and reached the ordinary timeout release at 0.0057 rad. This supports
a boot-loaded internal velocity limit as the cause; the firmware's internal
implementation was not inspected. The other five saved ceilings were subsequently
provisioned the same way, preserving their other boot settings.

## Test method and limits

The normal La Coulette backend was launched with
`./grip robot up --env la_coulette --robot all`. Its initializer explicitly homed
IDs 113, 447, 111, 345, 343 and 342 and reported `homing verified`.
After homing, the operator-authorized sequence used
`./grip mission gripper close --robot all`, waited for completion telemetry,
then `./grip mission gripper open --robot all`.

A separate read-only ROS recorder captured joint states at the configured
100 Hz, health at 5 Hz and driver logs. Service acknowledgements alone were not
used as completion evidence. Speed peaks are sampled values, not bounds on
the firmware's faster internal samples. Normal close timeout means torque hold;
normal open timeout means release to zero commanded torque with PD disabled.
These outcomes do not prove a mechanical endpoint or object retention.

## Final acceptance after provisioning all six

All six saved ceilings read 1500 after reboot. The other five originally saved
100 rad/s; 345 originally saved 10 rad/s. Configuration comparisons verified
only the ceiling changed. All were disabled (state 64), idle (mode 0), with
100 ms watchdogs before launching the backend. Some one-shot CLI reads timed
out and were repeated read-only; no motion command was retried on that basis.

The final initializer reported `homing verified` for all six at Unix timestamp
1789757293.397336. Close and open service acknowledgements were followed through
to the per-drive completion logs below. There were no drive faults during these
motions. All six were responsive and error-free afterward, with the driver
container running, healthy, and restart count zero.

| Drive | Peak homing rad/s | Peak closing rad/s | Close hold rad | Peak opening rad/s | Open completion rad | Open outcome |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| 113 | 1.66 | 2.44 | 0.7414 | 11.43 | 0.0065 | timeout release |
| 447 | 1.44 | 2.50 | 0.7332 | 11.75 | 0.0061 | timeout release |
| 111 | 1.73 | 2.43 | 0.7328 | 11.81 | 0.0031 | timeout release |
| 345 | 1.40 | 2.64 | 0.7370 | 11.57 | 0.0032 | timeout release |
| 343 | 1.41 | 2.34 | 0.7299 | 11.75 | 0.0032 | timeout release |
| 342 | 1.71 | 2.29 | 0.7310 | 11.25 | 0.0048 | endpoint criterion; holding zero target |

All close completions used the normal two-second torque-hold timeout. At the
post-open snapshot the released drives had relaxed to 0.0048–0.0180 rad, with
near-zero measured torque; 342 remained near 0.0048 rad with approximately
-0.14 Nm holding torque. No camera or manual jaw-gap measurement was made.
The backend was left running after this final opening; the telemetry recorder
was stopped. The full pipeline remains a separate issue because its trajectory
caches did not match the active configuration.

Evidence archive:
`/home/jonas/Documents/candle-validation-2026-09-18-velocity/evidence.tar.gz`.
It contains all three recordings, snapshots, per-drive configuration backups,
provisioning/readback logs, build/test output and the analysis scripts. The
initial failed attempt is preserved. Source and this report are archived beside
the evidence; the working changes remain uncommitted in `candle_ros2`.
