# MD20 release preparation — 2026-09-16

Branch: `feature/continuous-impedance-close`. These results cover the uncommitted
working tree, not a released revision. No commit, push, production-stack startup
or zero change was performed. Initial software checks used disabled drives;
Subsequent operator-authorized empty cycles on 113/447 and a bottle trial on 113
are documented below.
111 remained disabled throughout this release-preparation session.

## Completed

- Clean native-Docker Humble build and driver tests passed. The rebuilt image
  `candle-ros2:humble-validation` reports 47 passing test entries, including CTest
  wrappers: 43 C++ cases and the recorder's hardware-free self-test.
- Jazzy build of `candle_ros2` and the separate GRIP `grip_candle_gripper` package
  passed in `/tmp/candle-release-jazzy.ertV4e`. All driver checks and four real-ROS,
  simulated-drive initialization cases passed: valid startup without zero;
  rejection of an old driver before init; disable all selected drives after
  partial init; rejection of mismatched settings. Colcon reports 48 test entries
  because those four Python cases share one CTest entry.
- Three targeted GRIP lifecycle/command-routing tests passed. The full
  `tests/test_grip_lifecycle.py` run had 136 passes, one failure and six setup
  errors. All seven unsuccessful cases failed to create test directories under
  `data/motion_planning` with `PermissionError`; a targeted rerun confirmed this.
  Existing planning artifacts and their permissions were not changed.
- All 36 settings in the prepared GRIP profile matched parameters read from the
  actual bench driver. This was a parameter-only check, not production startup
  or an enabled-motor integration test.
- `/md/disable` returned confirmed success for 111, 113 and 447. Separate,
  exclusive-bus SDK reads found hardware state `64` (`0x40`, disabled) and
  `canWatchdog = 100` ms on each. No watchdog register was written.
- Graceful SIGINT and SIGTERM shutdown checks passed with the native driver
  executable as container PID 1. Each logged disabled-state confirmation for
  all three drives and exited with status 0. Drives were already disabled;
  these checks do **not** establish stopping behavior under load.
- The earlier bench command using `exec ros2 run ...` did not forward SIGINT;
  Docker killed that wrapper after its grace period (exit 137). Use the installed
  executable directly as PID 1 for standalone bench containers. Production's
  existing `exec ros2 launch ...` path was not started here.
- Both repositories pass `git diff --check`. At handoff the bench driver is
  stopped and all three drives are disabled.

## Preserved commissioning evidence

[111 speed measurements](2026-09-16-speed-111.csv) preserve nine earlier empty
gripper trials. Five used the final profile: closing/opening ramps 2/10 rad/s,
leads 0.04/0.14 rad, Kp 30, Kd 0.2, closing Ki 5, 2 Nm cap/object hold and
0.25 Nm closed bias. Targets are 0/0.75 rad and calibrated travel is
[-0.05, 0.75] rad. No zero was changed during speed tuning.

Over the 0.10–0.60 rad interval, final-profile median speed was 1.974 rad/s
closing and 9.909 rad/s opening. These are interval averages, not instantaneous
speed limits. Recorded opening position reached -0.01588 rad and peak torque
was 1.368 Nm. All five finished healthy and disabled; four used the requested
zero-torque opening timeout release, while one confirmed the open endpoint.
Timeout release must not be reported as confirmed endpoint arrival.

### Final-profile empty checks: 113 and 447

After the operator confirmed both mechanisms empty and clear, each completed
one inspected cycle followed by nine repeats. Every cycle closed, held for
10 seconds, opened, and confirmed disable. The same profile was used on both;
no limits, gains or zeros were changed. All 20 cycles completed without a
hardware/state fault, missed close, travel violation or recorder abort.

| Drive | Median close, rad/s | Median open, rad/s | Released position range, rad | Peak recorded torque, Nm | Peak motor temperature |
| --- | ---: | ---: | ---: | ---: | ---: |
| 113 | 1.980 | 9.273 | -0.000565 to +0.007089 | 1.312 | 33.48 C |
| 447 | 1.955 | 9.790 | +0.003385 to +0.008692 | 1.343 | 34.02 C |

Speeds use interpolated 0.10–0.60 rad crossings, as for 111, not whole-command
duration or instantaneous limits. Recorded travel stayed within [-0.05, 0.75]:
113 reached -0.011620/0.740721 rad and 447 reached -0.005446/0.744019 rad.
Per-cycle positions, torque, speed and outcomes are preserved in
[the empty-cycle summary](2026-09-16-empty-113-447.csv).

**Motion health passed; tight opening accuracy did not consistently pass.**
113 confirmed an open endpoint in 3/10 cycles and 447 in 4/10; the others used
the requested zero-torque timeout release, with near-zero torque verified.
After final disable, 1/10 readings for 113 and 7/10 for 447 exceeded 0.005 rad
from zero. For example, 113 repeat 02 was near +0.0034 rad before timeout,
then moved to +0.0071 rad after torque was removed. A confirmed powered endpoint
also does not guarantee the same position after disabling its PD hold.

Initial pre-test positions were +0.028875 rad (113) and +0.074127 rad (447),
so returning to the commanded zero necessarily differed from those initial
baselines. Among the nine subsequent repeats, cycle-to-cycle released-position
changes exceeded 0.005 rad twice on 113 and once on 447. Do not conflate that
comparison with absolute error from the open target.

Both mechanisms held around 0.736 rad, short of the settled 0.75 rad endpoint.
All closing outcomes were timeout torque holds, not confirmed closed endpoints:
113 held 0.544–0.560 Nm; 447 held 0.557–0.626 Nm. These checks therefore do not
validate the 0.25 Nm settled-endpoint bias or a 2 Nm loaded object hold.

At the end of the empty checks, fresh feedback showed positions 111 = 0.000000 rad,
113 = -0.000345 rad, 447 = +0.003375 rad, all responsive and disabled with
near-zero measured torque. The driver then stopped cleanly with another
disabled-state confirmation for every drive. No motion is scheduled.

### Final-profile bottle hold: 113

At 16:02 UTC, after operator clearance for the bottle in 113, one guarded
close/10-second hold/open/disable cycle completed. 111 and 447 remained disabled;
the profile and all zeros were unchanged.

- Contact inferred at 0.3664 rad; object-hold target reached at 0.4001 rad.
- Across the 10-second observation, measured torque averaged **2.00003 Nm**,
  range 1.99301–2.00502 Nm. Whole-cycle peak measured torque was 2.00740 Nm
  against the unchanged 2 Nm command cap and recorder's 2.1 Nm measured guard.
- Position changed inward from 0.40359 to 0.40769 rad during the hold; the final
  half-second mean was 0.407593 rad. No recovery restart, hardware/state fault
  or guard abort occurred. Telemetry alone does not establish grip security or
  distinguish bottle deformation from fingertip compliance.
- Opening used the configured two-second zero-torque timeout release, not
  confirmed endpoint arrival. Released mean position was -0.000556 rad, only
  -0.000156 rad from the initial baseline, with near-zero measured torque.
- Motor temperature started at 32.16 C and peaked at 46.11 C; MOSFET temperature
  peaked at 44.26 C. This short test is not a continuous-duty thermal rating.
- Final disable was confirmed for every drive; the bench driver then exited
  cleanly. No motion is scheduled. This was a static hold, not a deliberate
  disturbance/recovery or slip-force test; operator observation remains needed.

### Raw-trace archive

Raw copies remain under `/tmp/candle-closed-bias.tGtPAk`. Durable local archives
outside the repository were also created in
`/home/grip-pc/Documents/candle-validation-2026-09-16.s2xma9`:

- `111-speed-raw.tar.gz`: nine original CSV/JSON pairs;
  SHA-256 `bcbd5b84200956b2e17a54c35015bd2aa51a7b1bd735536f5d5ab7b4e7a418b9`.
- `113-447-empty-raw.tar.gz`: 20 CSV/JSON pairs plus the profile and recorder;
  SHA-256 `24318fe2b295d4d4e5316513c34f9d8abe60b20e9c22255958b27e9a5b99e747`.
- `113-bottle-raw.tar.gz`: the static bottle CSV/JSON pair, profile and recorder;
  SHA-256 `4be1aaad4d2ffe419c1848ee13eeb48f3062dbd3562254407c6a3f60287906ba`.

All three compressed archives passed integrity checks. These local archives are not
included in the Git change; the CSV summaries are not substitutes for raw traces.

## Integration prepared, not deployed

The separate GRIP repository's `src/grip_candle_gripper` forwards the tuned
settings, verifies the loaded parameters before init, preserves calibrated
zeros, and requests disable for all selected drives if initialization fails.
The existing services and startup path are reused; no new control wrapper runs
alongside the driver. Legacy PID configuration fields remain schema-compatible
but are no longer applied during impedance startup.

`src/grip_candle_gripper/docker/deps.repos` still pins the old driver commit
`2892ecc9734bf09a1e834c9d82f45792ce1f008e`. Do not deploy that old driver with
the new integration; its parameter preflight intentionally rejects the mismatch.
After driver review, commit and push, pin its exact tested SHA, rebuild the
production image and validate the namespaced startup with the scene clear.

## Remaining before release approval

1. Resolve the opening-position acceptance criterion: the requested passive
   zero-torque release does not consistently stay within 0.005 rad of zero.
   Do not silently widen tolerance, re-zero or replace that release with a hold.
2. Repeat loaded checks with varied bottle placement and thin foil, and test
   deliberate position-recovery behavior with operator setup. One static 113
   bottle hold passed the measured 2 Nm target; grip security and the other
   mechanisms/loads are not established by that single run.
3. Perform supervised disable/shutdown and communications-loss/watchdog tests
   from an enabled state. The configured 100 ms watchdog and unit tests do not
   prove physical expiry or stopping distance. Do not disable thermal protection.
4. Resolve the wider lifecycle test environment using isolated writable fixtures,
   not changes to production planning artifacts.
5. Review both repository diffs separately, commit/push the driver, then update
   and validate the GRIP dependency pin. Share the local raw archives if needed.

The guarded recorder is installed as `record_gripper_cycles.py`. It requires
explicit IDs and `--clear`, rejects an unsafe profile, preserves zeros and
existing output files, and stops on a fault or unconfirmed disable. Its `--clear`
flag is an operator confirmation, not automated scene validation.
