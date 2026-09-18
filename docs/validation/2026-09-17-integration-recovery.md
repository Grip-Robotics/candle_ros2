# GRIP integration, braking and persistent state — 2026-09-17

## Result and scope

Completed the cleared grippers-only integration test using the real GRIP launch,
initializer, health bridge, scene ID mapping, request builder and Zenoh transport.
Robot-arm services were never started. Driver changes remain on
`feature/continuous-impedance-close`; GRIP changes remain in its existing
`fix/trajectories_miroring` worktree. The unrelated mirrored-scene edits were
preserved. No commit, push, merge, production image replacement or dependency-pin
update was made.

The final driver completed the basic ordered cycle and two subsequent mixed-state
shutdown/restart tests. All test processes stopped; final exclusive SDK readback
confirmed every drive disabled (state 64, mode 0), watchdog 100 ms, no hardware
fault and near-zero torque. No zeros, targets, gains or travel limits were changed.

## Bugs fixed and failed attempts retained

- GRIP's health bridge stored its mapping in `Node._publishers`, clobbering ROS's
  publisher registry and raising `KeyError: 0` during destruction. Renamed that
  bridge-owned mapping; a real ROS construct/destroy regression failed before
  the fix and passes after it.
- The unconditional 0.25 Nm closing floor suppressed native PD braking. At
  15:13:42 UTC, unloaded 111 accelerated to a sampled 7.512 rad/s. The guard
  requested disable when position reached 0.750385 rad; braking then reached a
  sampled 0.842309 rad before stopping. The asynchronous stop confirmed after
  37.5 ms, and final readback returned in range with near-zero torque. This run
  **failed** and must not be presented as a passing floor validation.
- Fixed the shared controller, used by both close service paths: restore PD + I
  when closing velocity exceeds the ramp speed or position gets ahead of the
  ramp target by more than the endpoint tolerance. Apply the same condition to
  lower-bound anti-windup. The 0.25 Nm floor still applies to normal closing and
  closing holds, but **not when braking takes priority**. New bidirectional
  feedback-replay and light-jaw dynamic regression tests fail on the old floor
  and pass on the corrected controller. This is not a certified hard speed cap.
- The test recorder originally demanded zero effort after every opening. A
  confirmed open endpoint legitimately retains PD, unlike the zero-torque
  opening timeout. Corrected that assertion after the 15:19:08 attempt; no
  opening-control behavior was changed.
- The 15:19:48 attempt stopped 113 too early: its valid near-endpoint recovery
  restarted the two-second close deadline. The recorder now waits at most five
  seconds for a completed hold stable for 0.5 seconds. Driver deadlines and
  physical guards were not relaxed. A separate 4 rad/s closing-speed test guard
  now stops an anomalous acceleration before it can reach the travel boundary.
- Simulated queued disabled-health frames exposed a startup race: a pre-init
  health message can arrive after init's acknowledgment. The initializer now
  waits through only that known disabled-latch transition, retains all physical
  checks, rejects other faults immediately and never extends its startup deadline.

## Persistent state behavior

GRIP's existing health bridge writes per-CAN-ID position, velocity, torque,
observed open/closed/partial state, health and timestamps every second and on
graceful exit. It reuses the existing process; no new package, proxy command
node or dependency was added. Writes use a temporary file, fsync, atomic replace
and directory fsync. Unselected drives' records are preserved.

Production storage is `/var/lib/grip/candle/grippers.json` in the named Docker
volume `grip-candle-gripper-state`, independent of the container lifecycle.
The isolated test used the equivalent persistent host bind under
`/tmp/candle-grip-integration.VrnUwa/state/`, not the production volume.

Startup checks the actual driver parameters, discovers/disables selected drives,
requires fresh valid feedback and compares saved/live positions before enabling.
It never replays a saved torque/command or re-zeros. Differences over 0.05 rad,
corrupt/incompatible files, invalid saved observations, or missing/faulted live
feedback stop readiness and automatic motion. Missing records use the calibrated
live reference, as on first commissioning. The health bridge cannot overwrite
old evidence until startup is ready; a failed state monitor shuts down the launch.

Only grippers outside the configurable 0.02 rad startup-open band receive an
open request. Targets remain 0.00/0.75 rad; driver endpoint tolerance remains
0.005 rad. The wider **classification band** accounts for passive release rebound
and is not a claim of precision. Opening is observed beyond its two-second
deadline, with fresh healthy settled feedback required before readiness.

Snapshots do not prove absolute position after a cold power cycle, nor detect
every possible whole-turn movement while power is off. Firmware zero/turn
persistence was **not physically power-cycle-tested** here. A suspicious reference
change is rejected, not repaired by guessing. Sudden power loss retains only the
last periodic checkpoint. A restart may release a held object; the physical
scene must be clear/supported before starting the pipeline.

## Hardware results

| Start UTC | Test | Result |
| --- | --- | --- |
| 15:21:37 | 111 → 113 → 447 close, hold 3 s, open together, restart | Passed |
| 15:29:31 | Same cycle; then save 111/447 closed, 113 open; restart twice | Passed |
| 15:33:59 | Repeat with final GRIP startup safeguards and queued-health fix | Passed |

Both mixed-state tests saved 111/447 closed and 113 open. The next startup
issued opening commands to **447 and 111 only** (scene order); 113 stayed at its
measured open position. The following already-open startup issued **no open
commands**. No close command was replayed. Shutdown logs contained no child
process failure/traceback. The final snapshot has `clean_exit: true` and all
three classified open with the intentional disabled health latch.

Final cycle's last 0.5-second hold/open windows:

| ID | Close hold rad | Hold Nm | Open rad | Open Nm |
| --- | ---: | ---: | ---: | ---: |
| 111 | 0.740587 | 0.388123 | 0.000046 | -0.000103 |
| 113 | 0.734026 | 0.660532 | 0.000664 | -0.000271 |
| 447 | 0.733847 | 0.677253 | 0.003658 | -0.109653 |

447's open window is a confirmed PD endpoint hold, not a timeout release.
Across final-run samples in the 0.1–0.6 rad sweep, mean closing speeds were
2.027/2.053/1.993 rad/s for 111/113/447; sampled peaks there were
2.716/2.283/2.807 rad/s. Simultaneous-opening mean speeds in the same window were
9.382/9.410/9.316 rad/s. All sampled travel stayed in [-0.05, 0.75] rad; maximum
absolute measured torque was 1.319 Nm and maximum motor temperature 33.34 °C.

Tight passive opening accuracy is still **not generally passed**: the preceding
mixed-state run ended 447's selective opening at approximately 0.01275 rad.
The startup band intentionally classifies this as open. Empty closures used
captured timeout torque; this is not a new hardware validation of the settled
0.25 Nm closed-endpoint mode or of bottle/foil retention after the braking change.

Final exclusive hardware readback at 15:34:40 UTC:

| ID | Position rad | Torque Nm | Motor °C |
| --- | ---: | ---: | ---: |
| 111 | 0.000077 | 0.003624 | 32.72 |
| 113 | 0.000729 | -0.000283 | 32.80 |
| 447 | 0.001227 | -0.002792 | 32.83 |

## Software checks and build identity

- Jazzy and Humble: 49 C++ test cases plus recorder self-test; colcon reports
  53 entries including suite wrappers, no failures/errors/skips.
- GRIP ROS integration: 11 tests covering initialization, selective opening,
  saved-reference disagreement, corrupt state, partial init/open failure,
  publisher cleanup, durable state merging and invalid/stale feedback.
- GRIP lifecycle suite: **143 passed** in an isolated writable copy with fake
  Docker, no USB and no network. An earlier read-only full-suite invocation was
  invalid for tests creating runtime fixtures (14 failures, 6 errors); rerunning
  in a disposable writable copy resolved them without unrelated source changes.
- `git diff --check` passes in both repositories.

Jazzy driver SHA-256:
`260fcb6b1a497383a5e34e69ac820841b4ce3d781f4f2e3e2a7e537058c6d639`.
Humble driver SHA-256:
`78d040cf926d2530d7ad8a7b0b0134ed75c4ccf3d20c6ee96a4bd243b34d66a0`.
Unchanged commissioning profile SHA-256:
`d2abe821f1c0db7da2eced865a50625cfa835e27c83f727e1b0dad9f58ad8254`.

Artifacts in `/home/grip-pc/Documents/candle-validation-2026-09-17.iCSJS6/`:

- `grip-integration-braking-recovery-raw.tar.gz`: all six attempts from this
  continuation, including failures, CSV/JSON/launch logs, final recorder/Compose
  override, persistent snapshot and GRIP test output. SHA-256
  `fe32d70f564edbb4bb4363579301ab465c3bc005721d96fcdd15201b4e06572e`.
- `grip-integration-braking-recovery-source-build.tar.gz`: driver/controller
  source and tests, GRIP integration sources/configuration, both driver binaries
  and distribution build/test logs. SHA-256
  `522a48ca3ec2cf09717391d1f3bd9bbeae6fac4b58a39d7860b68d6a428e16d1`.
  The later YAML comment-only relocation does not change its configuration values.
