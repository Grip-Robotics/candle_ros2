# Parallel startup homing validation — 2026-10-15

Both `/home/jonas/grip` and `/home/jonas/candle_ros2` are on
`dev/slow-belt+homing`. Driver base: `d482a81a64b8f5376d8b36ba69f11251568a3acb`.
The implementation is committed locally on this branch in both repositories;
the published dependency pin is unchanged.

## Behavior

One `/grip/candle/md/home` request selects all six motors. The existing single
ROS executor serializes bus transactions while independent motor controllers
run concurrently. All motors pass disabled preflight/configuration before enable.
Phase barriers coordinate opening, closing and reopening. Motors that finish
first remain disabled, with continuous release-settling checks and the original
release deadline. Each motor learns its own opening/closing endpoints.
A member failure or explicit disable aborts the group and invalidates every
member's calibration. Automatic retries remain blocked after a group failure.

## Software verification

171 C++ cases passed, including nine group-homing cases; the recorder self-test
and all 21 GRIP initializer/health tests passed. Cases cover invalid/duplicate
requests, concurrent requests, preflight failure, partial enable failure,
communication loss, failed release disable, explicit cancellation, total deadline,
passive rebound with an unchanged release deadline, and staggered completion with
separate calibrations. The final strengthened rebound test was rebuilt and all
nine group cases rerun successfully. `git diff --check` passed in both repositories.

## Hardware evidence

Selected drives: `[111, 447, 113, 345, 343, 342]`, across two grippers.
Only the CANdle service was replaced; no arm-motion or picking request was sent.

The first live attempt stopped during the closing-release barrier: motor 345
resumed passive rebound after its first settled dwell. All six were confirmed
disabled on shutdown. The barrier was corrected to continue the existing settling
check while waiting, without increasing tolerance or restarting the deadline.
The regression test covers this behavior.

The corrected startup completed at 17:22:58 UTC in **5.823 s**; a second explicit
group homing completed at 17:24:13 UTC in **5.051 s**, returning six `true` results.
Both completed all three phases and final unloaded release. The earlier same-day
sequential baseline took **21.663 s** from first home request to final motor
completion (14:35:01–14:35:22 UTC). This is about 16 seconds saved on that comparison;
starting positions differ, so it is not a controlled benchmark. Startup reboots
remain sequential and take about 18 seconds for six drives.

In the first corrected cycle, all six motors showed velocity in the requested
direction in the first published sample after each group phase started
(approximately 30 ms after the phase log). This confirms overlapping measured
motor motion, not sub-millisecond synchronization. The longest joint-state
publication interval during homing/initialization was 68.3 ms, below the configured
250 ms feedback timeout. The final health read reported all six responsive,
no errors and motor temperatures around 22–25 C. The container was healthy.
This is encoder/driver evidence, not an independent visual inspection of the jaws.

| Motor | First closed stop (rad) | Repeat closed stop (rad) |
|---|---:|---:|
|111|0.94792|0.94719|
|447|0.89830|0.89972|
|113|0.85040|0.85055|
|345|0.88246|0.88307|
|343|0.72404|0.72599|
|342|1.08126|1.04848|

Motor 342 showed more reference/rebound variation: unloaded reopen was 0.02711 rad
in the first cycle and 0.00065 rad in the repeat. Its loaded open-to-closed stroke
was 1.11896 and 1.11298 rad, respectively. Both calibrations passed the unchanged
checks; these readings do not establish mechanical repeatability independently.

## Local image and reproduction

Tested image: `grip-candle-gripper:parallel-homing-20261015`, also tagged locally as
`grip-candle-gripper:jazzy` for normal startup. Image ID:
`sha256:39a4ca58c4d45b50ec8934316cb45be7bdd3596e7820555e67fa30bcc77730d2`.
Previous default image retained as `grip-candle-gripper:before-parallel-homing-20261015`.
The driver container remains running, healthy, after the completed open/release.
Production driver and initializer hashes were checked against the edited files.
Normal `./grip` startup reuses the existing images and launches the robot services
with `--no-build`; all required robot-stack images were verified present before
committing. A computer restart preserves these local Docker images. This confirms
parallel homing for normal startup on this machine, without a forced rebuild.

Until the companion driver change is published and GRIP's `deps.repos` pin is
updated, rebuild from the sibling checkout (the default pinned-source Dockerfile
does not contain parallel homing):

```bash
GRIP_CANDLE_LOCAL_IMAGE=grip-candle-gripper:jazzy docker compose \
  -f compose.yaml -f compose.local-candle.yaml build grip-candle-driver
```

Raw logs and feedback are in `/home/jonas/grip/logs/homing/parallel-20261015/`:
`first-run.log`, `successful-runs.log`, `feedback.jsonl`, `repeat-feedback.jsonl`,
`sequential-baseline.txt`, `software-tests.log`, and `barrier-tests.log`.
