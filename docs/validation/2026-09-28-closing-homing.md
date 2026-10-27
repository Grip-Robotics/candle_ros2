# Open-close-open homing: software and physical validation

The existing `/md/home` service now completes an opening search and disabled
release/zero, a closing search and disabled release, and a reopening search and
disabled release. Only a return within the original zero tolerance, followed by
successful normal released-mode setup, makes the drive ready.

Each drive learns its loaded closed-stop position as its normal closing target.
`homing_travel_margin_rad` defaults to 0.5 rad beyond that endpoint in the closing
direction; the configured opening-side bound is retained for normal motion.
Homing has no preset position or travel bound: it measures those limits. Torque,
timing, feedback, direction and final-zero checks remain active. The obsolete
`homing_max_travel_rad` parameter is removed. Calibration is kept in memory,
never written to firmware position limits or flash.

The calibration is discarded on rehoming, explicit zero, and communication loss,
including a failed disable readback. Incomplete calibration blocks normal motion
and enable. Explicit disable cancels homing. `/home` rejects
`init_devices_zero=true` so initialization cannot silently change a learned frame.

Validation on 2026-09-28:

- The complete ROS Jazzy package built successfully in an isolated Docker
  container using `grip-candle-gripper:jazzy`, with `--network none`, no hardware
  device mounts, a read-only source mount, and a separate build/install directory.
  An incremental rebuild after the recovery/invalidation review also passed.
- All 76 tests in the position tracker, drive state, impedance control, and
  gripper homing suites passed. The 17 focused homing tests cover both directions,
  independent endpoints, exact margins, invalid transitions, missing stroke,
  feedback/torque/time/direction guards, searches beyond the old 0.85 rad bound,
  unrestricted homing rebound, release settling, and failed return to zero.
- The cycle recorder's hardware-free self-test passed.
- Launch Python syntax, YAML profile parsing/default margin, and
  `git diff --check` passed.

The C++ tests were compiled directly with the repository's bundled GoogleTest
because the runtime image does not include `ament_cmake_gtest`:

```bash
g++ -std=c++20 -O1 -Wall -Wextra -Wpedantic -pthread \
  -Iinclude \
  -ICANdle-SDK/commons/googletest/googletest/include \
  -ICANdle-SDK/commons/googletest/googletest \
  test/impedance_close_test.cpp test/position_tracker_test.cpp \
  test/drive_state_test.cpp test/gripper_homing_test.cpp test/impedance_home_test.cpp \
  CANdle-SDK/commons/googletest/googletest/src/gtest-all.cc \
  CANdle-SDK/commons/googletest/googletest/src/gtest_main.cc \
  -o /tmp/candle-homing-tests
/tmp/candle-homing-tests
python3 scripts/record_gripper_cycles.py --self-test
```

Build artifacts and logs from this run are in
`/tmp/candle-closing-homing-20260928/`. Existing SDK/compiler warnings remain.

## Physical test through GRIP

The user authorized physical testing and confirmed the workspace was clear.
A separate image, `grip-candle-gripper:closing-homing-unbounded-20260928`, was built
from the local working tree and started with GRIP's production driver launch and
initializer. Its source hashes were verified against the embedded manifest.
The stock image was retained; the default GRIP dependency pin was not changed.

All six drives completed open-zero -> close-calibrate -> reopen in one sequential
startup at 14:04 UTC. Fresh health feedback then showed all six responsive and
without errors. The measured normal bounds were:

| Drive | Loaded closed stop (rad) | Normal upper bound (rad) | Released reopen (rad) |
| --- | ---: | ---: | ---: |
| 113 | 0.94673 | 1.44673 | 0.00004 |
| 447 | 0.83080 | 1.33080 | 0.00000 |
| 111 | 0.77823 | 1.27823 | 0.00012 |
| 345 | 0.80822 | 1.30822 | 0.00012 |
| 343 | 0.84611 | 1.34611 | -0.00012 |
| 342 | 0.77010 | 1.27010 | -0.00019 |

The normal lower bound remains -0.05 rad and is inactive during homing. No
firmware position limits or flash were written; the learned bounds are in memory.

Earlier trials exposed two relevant conditions:

- With the original search cap still present, 113 exceeded 0.85 rad while opening
  and then while closing. Its measured closed stop after removal was about
  0.9467 rad, demonstrating why the old bound prevented calibration.
- Drive 345's first full sequence returned -0.04149 rad from its initial zero.
  Both loaded stop readings had approximately 1 Nm effort; the different opening
  positions suggest path-dependent mechanics. Calibration correctly failed and
  all drives were disabled. One complete repeat from its settled position passed
  without weakening the zero-repeatability check. The physical cause is not proven.

An additional normal close/open test on 113 reached 0.93703 rad without the old
travel fault. Normal opening then reached its existing 0.5 s timeout at 0.0066 rad,
released torque, and settled at 0.02228 rad, outside GRIP's 0.02 rad open band.
That separate normal-motion test failed; it is not a homing success claim or
proof that a picking mission is ready. All six drives were disabled, the driver
was stopped, and independent register reads confirmed disabled state 64 and
IDLE mode 0. No robot arms or picking mission were run.

Attempt 2 also inferred an opening stop while measured effort remained near zero.
Command registers were not recorded, so that anomaly remains unexplained. Later
successful homing cycles recorded approximately +/-1 Nm at the inferred stops.
Stop detection remains sensorless: an object, friction or jam can resemble a
mechanical stop. These tests establish encoder/control behavior, not physical
jaw-gap or object-retention measurements.

Full driver logs, telemetry, readbacks, build provenance, compose overrides and
normal-cycle results are under
`/home/jonas/grip/logs/gripper-homing/20260928T135346Z/`.
