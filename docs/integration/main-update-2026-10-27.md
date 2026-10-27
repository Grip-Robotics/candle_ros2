# CANdle driver integration queue — 2026-10-27

`main_update` is the staging target, created from `main` at
`307c8a5a86efbb881f08b3e9721b5614534962a7`. At creation it contains the
same runtime code as `main`, plus this guide. No code PR has been merged by
this organization task. The complete reviewed sequence is intended to land
here before a later `main_update` → `main` PR.

## Merge order

1. [#5 — Health, recovery, and bounded transport foundations](https://github.com/Grip-Robotics/candle_ros2/pull/5)
   - Head: `feature/gripper-health-monitoring` (`d7c5cd0`).
   - Base: `main_update`. Fifteen prerequisite commits; 19 changed files.
   - Introduces the historical gripper/health foundation and tooling.
2. [#4 — Bounded continuous impedance control](https://github.com/Grip-Robotics/candle_ros2/pull/4)
   - Head: `feature/continuous-impedance-close` (`8665938`).
   - Review base: `feature/gripper-health-monitoring`. Two commits; 28 changed files.
   - Replaces staged control and adds damped homing and confirmed disable.
3. [#6 — Calibrated homing and normal opening](https://github.com/Grip-Robotics/candle_ros2/pull/6)
   - Head: `fix/normal-opening-no-position-tolerance-2026-10-05` (`c619c5d`).
   - Review base: `feature/continuous-impedance-close`. Two commits; 33 changed files.
   - Includes `e05c5c5`: open-close-reopen calibration, normal opening,
     fault capture, verified zero readback, and SDK transport validation.
   - Includes `c619c5d`: release ordinary opening on its deadline without
     accepting an absolute position as a homing result.
4. [#7 — Verified startup reboot and opening obstruction band](https://github.com/Grip-Robotics/candle_ros2/pull/7)
   - Head: `fix/startup-reboot-opening-band-2026-10-05` (`adddd70`).
   - Review base: `fix/normal-opening-no-position-tolerance-2026-10-05`.
   - One commit; 11 changed files.
5. [#8 — Command readback verification and startup diagnostics](https://github.com/Grip-Robotics/candle_ros2/pull/8)
   - Head: `fix/command-verification-2026-10-05` (`d482a81`).
   - Review base: `fix/startup-reboot-opening-band-2026-10-05`.
   - One commit; seven changed files. Advances SDK reply-type validation.
6. [#9 — Simultaneous gripper homing](https://github.com/Grip-Robotics/candle_ros2/pull/9)
   - Head: `feat/simultaneous-gripper-homing` (`6c02f31`).
   - Review base: `fix/command-verification-2026-10-05`.
   - One commit; six changed files. Per-drive jobs share phase barriers and
     abort together on failure.

The numbered titles specify ordering; GitHub PR numbers are not chronological
merge order because #4 already existed.

## How to integrate

- Merge #5 into `main_update` first.
- After each prerequisite lands, retarget the next PR's base to `main_update`,
  confirm its diff still contains only that stage, and merge it there.
- Use merge commits. Do not squash/rebase the chain: downstream comparisons
  rely on the existing commit ancestry. Keep prerequisite branches until their
  dependent PRs have been retargeted. Do not merge a PR into its feature-branch
  review base.
- Intermediate stages are for review, not deployment. In particular, #4's
  [zero-ACK review finding](https://github.com/Grip-Robotics/candle_ros2/pull/4#discussion_r4048645291)
  is fixed by #6: `/md/zero` and `init_devices_zero` verify the encoder frame
  with `readZeroPosition` while disabled. The historical review thread is
  intentionally retained rather than marked resolved on an unchanged commit.
- After all six stages land, verify that runtime sources match `6c02f31`, rerun
  the driver/integration checks, and open the final `main_update` → `main` PR.
  An existing successful homing test does not validate every possible grasp,
  power fault, or physical installation.

## Branches outside the queue

- `dev/slow-belt+homing` is the active local checkout and remains untouched.
  Its driver tip is the same `6c02f31` published by the final PR branch.
- `diag/soak-fault-capture-20260930` and `fix/normal-opening-10rads-20261001`
  both point to `e05c5c5`, already included in #6. They need no duplicate PR.
- `feature/gripper-health-monitoring` locally is older (`19a11b9`) than its
  remote review head (`d7c5cd0`). The queue uses the remote head and does not
  move the user's local branch.
- `dev/usb-serial-selection` and the batched-read/timeout work are already in
  `main` through #2 and #1; no duplicate integration is needed.
- `pre-grip` / `soft-close` are older development lines. `pre-grip` contains
  divergent commits, including #3, which are not ancestors of the tested tip.
  They are not blindly merged into this queue; any still-needed behavior needs
  a targeted comparison against the current implementation.
- Local `fix/docker-temperature-log` (`7e00b33`) is based on that older line.
  Leave it separate: the current driver already has temperature diagnostics,
  but this task does not claim that the old logging change is patch-equivalent.
- `vendor/candle-sdk-transport-20261001` (`d1eb6ca`) and
  `vendor/candle-sdk-reply-audit-20261005` (`7e2047e`) contain **SDK history**,
  not ROS driver history. Keep these published refs for submodule fetches.
  Their changes enter through the `CANdle-SDK` gitlink in #6 and #8; never merge
  those branch roots into `main_update`.
- No branches were deleted, force-pushed, or automatically merged.

## Validation and GRIP dependency

All adjacent stages were checked for ancestry. An offline local merge rehearsal
of the six stages preserves the exact `6c02f31` source tree, with only this
integration guide added. This rehearsal does not merge the GitHub PRs.

A fresh software run on 2026-10-27 used the existing source-verified test image
`grip-candle-gripper:simultaneous-test-20261027`, with no USB devices and
`--network none`:

- `colcon test-result`: **181 tests, zero errors/failures/skips**.
- The nested GRIP initializer suite reports **21 Python cases passed**.
- Counts are reported as emitted by the test runners, not added together.
- `git diff --check` passes for the final driver delta.

These checks cover the complete driver tip, not independent builds of each
historical intermediate revision. No hardware was operated for queue setup.
The earlier same-day live test of the final pinned driver completed six-drive
homing in **4.532 s**, with overlapping motion in all three phases and healthy
final feedback. That is encoder/driver evidence, not independent visual
measurement.

[GRIP PR #214](https://github.com/Grip-Robotics/grip/pull/214) already pins
`6c02f319b83c2b933ab6f32d116973ca17bfc97c`, so it can fetch/build the tested
source before these driver PRs are merged. Keep the exact pin while reviewing
this queue; there is no need to replace it with a moving branch name.

Local evidence for this organization task is under
`/home/jonas/grip-worktrees/candle-integration-validation-20261027/`.
