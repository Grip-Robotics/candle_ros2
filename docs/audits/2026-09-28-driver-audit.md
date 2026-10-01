# CANdle driver audit and physical GRIP validation — 2026-09-28

The audit produced driver, SDK transport, and GRIP integration fixes. The software
regressions pass, and the revised opening sequence completed repeated physical
cycles. **The six-drive system is not yet qualified for a picking mission:** drive
447 stopped producing the requested opening effort, and 343 still failed the
original-zero check after the full passive-release deadline on its final retest.
All drives were left disabled.

## Implemented fixes

- Persistent disable/fault authority across communication recovery, including a
  failed asynchronous disable during homing or normal opening. Later communication
  loss cannot silently re-arm homing after that fault. A failed zero/readback
  invalidates the reference and leaves the output disabled.
- Validate device IDs before narrowing (`65649` formerly aliased `113`), finite
  float-register values, gains, limits and complete array shapes before writes.
  Invalid replacement targets preserve the existing command; partial configuration
  failures stop the affected drive. Fault and temperature checks cover ordinary
  enabled commands as well as controller jobs.
- Opening and closing homing have no preset position/travel bounds. Opening
  establishes the unloaded zero; closing learns the loaded closed stop and the
  normal guard 0.5 rad beyond it; reopening must return to the original zero.
  Sensorless stop detection now requires measured directional effort throughout
  the dwell, and command/state readback before accepting a reference. Release
  waits for confirmed hardware disable without blocking other drives.
- Normal opening uses the calibrated powered opening stop and disabled release,
  verifies the original zero without re-zeroing, and ends with zero commanded
  effort. Passive release has its own 3 s deadline and requires a complete dwell
  at the original zero; an earlier off-zero plateau does not terminate release. It replaces the previous 0.5 s ramp/release for calibrated grippers.
  An already enabled impedance drive receives the new command directly; periodic
  command/gain/state/device-ID readback detects overwritten commands. Learned
  travel guards remain enforced during normal opening and release.
- SDK MD writes reject transport errors even with ACK-shaped payloads. Reads
  validate entire replies before updating cached registers. Packed replies check
  length, CRC, sequence and drive identity; oversized frames cannot overflow or
  wrap lengths. USB short transfers fail; synchronous and asynchronous bus
  transactions are serialized. PDS parsing rejects truncated data, and ROS modules
  do not publish fabricated fresh values after failed reads or duplicate setup.
- GRIP readiness now follows fresh, complete selected-drive health after successful
  initialization. Startup accounts for the complete homing sequence and unloaded
  torque. Pickup response arrays use `all(success)` instead of truthiness of a
  nonempty array. Shutdown tolerates only the specific native already-shutdown
  race. A local-source Compose build includes the dirty SDK and embeds source
  hashes; the published dependency pin is unchanged.

## Software verification

- Full ROS package build and installation with the repository's
  `BUILD_TESTING=ON`: **134 C++ tests and the recorder self-test passed**, all 10
  CTest targets passed. This includes 13 actual MdNode callback tests.
- Four failed-disable/recovery regressions fail without the one-line latch fix
  and pass with it. Other callback tests cover invalid IDs/numbers/arrays, failed
  zero, replacement commands, uninterrupted opening and periodic readback faults.
- **25 transport tests passed under ASan, UBSan and leak detection.**
- **18 GRIP regression tests passed**: 14 initialization/health/shutdown, two pickup
  result tests and two CLI/lifecycle tests. No network or USB was exposed to these
  software test containers. Parent/SDK/GRIP `git diff --check` passed.

## Physical tests through GRIP

The user authorized physical testing and confirmed clear grippers. Testing used
GRIP's production launch, initializer and ROS services, direct USB CANdle adapter
208337A64D41, with sequential actuation and fresh feedback/health monitoring.
Torque, time, temperature, reference and feedback checks were retained. There were
no firmware position-limit or flash writes, arm motions or picking missions.

The initial audited image calibrated 113 and 447, then correctly rejected 111's
-0.00755 rad released return. A complete repeat from its settled position passed
all six. The calibration measured these loaded stops:

| Drive | Closed stop (rad) | Normal upper guard (rad) | Powered opening stop (rad) |
| --- | ---: | ---: | ---: |
| 113 | 0.94121 | 1.44121 | -0.03808 |
| 447 | 0.83080 | 1.33080 | -0.03440 |
| 111 | 0.79364 | 1.29364 | -0.04859 |
| 345 | 0.80396 | 1.30396 | -0.03881 |
| 343 | 0.85109 | 1.35109 | -0.04870 |
| 342 | 0.77324 | 1.27324 | -0.03973 |

The normal opening-side guard is min(-0.05, powered opening stop -0.005) rad for
these positive-closing drives. It is not applied to homing searches.

113 then passed one normal close/open cycle. 447 closed, but its normal opening
remained near 0.77665 rad with only millinewton-metre measured effort and timed out.
All six disabled. Independent reads found target torque -1 Nm, Kp 0, Kd 0.5,
max torque 2 Nm and bus voltage 34.21 V; these reads were after disable and do not
prove applied output during the failed interval. Startup command/state readback
had passed. A subsequent full homing attempt also timed out on 447 near 0.77589
rad with near-zero measured effort. No real hardware/encoder/bridge/communication
fault was reported. The preexisting Warning Torque remained. `miscStatus` is not
readable through the legacy tool on this firmware and was not used as evidence.
The cause of missing output is unresolved; a firmware/output-stage issue remains
an inference, not a diagnosis. No further 447 motion attempts were made.

The `audit-final-20260928` revision (continuous opening transition plus periodic
readback, before the later passive-settling fix) calibrated the other five drives and attempted three cycles per drive:

| Drive | Completed cycles | Maximum final released-zero error among passes (rad) |
| --- | ---: | ---: |
| 113 | 3 | 0.000192 |
| 111 | 3 | 0.000805 |
| 345 | 3 | 0.000192 |
| 343 | 2 | 0.000345 |
| 342 | 2 | 0.000153 |

**13 cycles passed; the 14th failed; the 15th was not attempted.** On 343's third
opening, the powered stop was -0.05319 rad (within 0.005 rad of its newly calibrated
-0.04890 rad). After confirmed disable it settled at -0.00633 rad, outside the
unchanged 0.005 rad original-zero check. Its failure snapshot independently showed
state 64 / mode 0 / target torque 0 / measured torque 0.01103 Nm. The test disabled
all five and verified GRIP's ready marker had disappeared. No tolerance was
widened and no normal opening changed the encoder zero.

Continued recording showed that 343 returned inside tolerance about 2.12 s after
that early failure and then near -0.0003 rad while still disabled. The release
controller was completing on an intermediate unloaded plateau. The fix gives
passive release a separate 3 s deadline and requires the original-zero tolerance
throughout its dwell. It also excludes the previous out-of-band sample from a
new in-band settling window. All 419 recorded samples now replay successfully,
completing after approximately 2.81 s of release with no commanded effort.

The final `audit-release-window-20260928` image was then tested physically on 343
alone, keeping the other five disabled. Homing passed: closed stop 0.85792 rad,
upper guard 1.35792 rad, powered open stop -0.04702 rad, normal lower guard
-0.05202 rad. One close/open cycle passed, including a 1.93 s passive release;
its final released position was -0.000575 rad. The second cycle reached the
powered stop at -0.05012 rad, but remained -0.00721 rad from the original zero
through the complete 3 s release deadline. It stopped with independently read
state 64, mode 0, Kp/Kd/target torque zero and measured torque 0.00174 Nm. The third
cycle was not attempted. Readiness was removed and disable confirmed. This
remaining repeatability failure was not hidden by re-zeroing or looser tolerances.

Each successful cycle required fresh healthy feedback, a closed position within
0.02 rad of the calibrated loaded stop, then released opening within 0.005 rad,
low velocity/torque, and a further unloaded hold. These are encoder/control
checks, not proof of jaw geometry, object capture or retention. They are short
cycles, not qualification of long-duration loaded holding or power-cycle recovery.

Final independent tool reads confirmed **all six state 64 / IDLE mode 0**, low
measured torque and no reported real hardware/encoder/bridge/communication faults.
Motor temperatures were approximately 31–33 C and MOSFET temperatures 37–42 C.
447 remains near 0.776 rad rather than open. Driver and recorder containers are
stopped. The existing Zenoh router and unrelated grid generation were preserved.

## Original mission and remaining work

The original mission's 113 failure was an out-of-travel stop after repeated close
hold/recovery behavior, with fixed target 0.75 and upper guard 0.85 rad. The later
robot_0 group-open failure during a pick using 447 was a consequence of 113 being
faulted, distinct from 447's missing-effort issue in this audit. The new measured stroke exceeds that
old guard. The mission also had a separate 345 opening-resistance fault. Full
causal evidence is in the GRIP mission diagnosis linked below; the new tests do
not retroactively prove the mechanical causes of those older failures.

Before mission qualification, investigate 447's missing applied effort and
343's remaining unloaded reference repeatability. Inspect mechanical settling and
reference consistency,
and capture firmware/output behavior for 447 before choosing reset/firmware or
hardware remediation. PDS hardware, bus-unplug/power-cycle recovery, real-object
retention and extended loaded holds were not physically exercised. The mission
executor still needs completion/fault handling beyond a service-acceptance ACK;
the boolean-array fix alone does not provide that behavior.

## Reproduction and evidence

From `/home/jonas/grip`, build current local sources without starting hardware:

```bash
GRIP_CANDLE_LOCAL_IMAGE=grip-candle-gripper:local \
  docker compose --profile robot -f compose.yaml -f compose.local-candle.yaml \
  build grip-candle-driver
```

Starting that driver launches physical homing. Do not use the five-drive diagnostic
override for a picking mission; it deliberately excluded the faulted 447.

- Audit artifacts: `/home/jonas/grip/logs/gripper-audit/20260928T141728Z/`.
- Original mission diagnosis:
  `/home/jonas/grip/logs/missions/20260928T132058.572807Z-2127959/gripper-diagnostics/diagnosis.md`.
- Five-drive cycle image: `grip-candle-gripper:audit-final-20260928`,
  `sha256:e9fac6da3be15cfb2facbf67d3305ca5c81bb192fcba35bff0d202cb2703f62a`.
- Final 343-tested image: `grip-candle-gripper:audit-release-window-20260928`,
  `sha256:6ca0cd41dd7a8b847d8b41dca1188ec4a2ce4ae1d0314fd449aca8927c2c9e9c`.
- Tested `src/md_node.cpp` SHA-256:
  `c6d7e4602cebfb7f7104722221ccd9b948969aae4964f537c92c757a5dfbc440`.
- SDK base: `a4fb4e16d9c38ff8f1c9efb94559e7b776a09f97`, with audit working-tree
  changes captured in `sdk-after.patch`. Parent base:
  `8665938ab5e5ccba3d0f94b70b4790526eab6d30`.
- The subsequent reviewed image includes the narrowly scoped Python shutdown-race
  fix, verified by three real SIGINT shutdown smoke tests. The final image adds
  the bounded passive reference dwell. All final driver/SDK/GRIP runtime sources
  match the working tree. Exact image IDs, comparisons and tested revisions are in
  `provenance.json`. The stock `grip-candle-gripper:jazzy` image and dependency
  pin were retained.
