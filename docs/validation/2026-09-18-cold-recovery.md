# Physical power-cycle recovery — 2026-09-18

Status: **cold-start recovery failed its reference preflight after the physical
power cycle**. Saved state survived, but the motor position coordinates no longer
match it. Recovery motion was not attempted; all motors remain disabled.
No zero, gain, travel limit or production code changes.

Uses the real GRIP launch/initializer/health bridge, persistent host state file,
isolated Zenoh transport and final braking-fixed driver SHA-256
`260fcb6b1a497383a5e34e69ac820841b4ce3d781f4f2e3e2a7e537058c6d639`.

## Preparation

The operator confirmed all grippers empty and clear, with power still on.
Attempt 01 at 08:57:35 UTC stopped before motion because 111 read 0.631847 rad
against a previously saved near-zero position. After the operator reported 111
fully open, attempt 02 read -0.026730 rad and stopped on the harness's overly
strict initial-open-band requirement. Neither attempt enabled a motor or
changed the stored reference.

The harness was corrected to use the existing 0.05 rad saved/live reference
check and normal selective startup opening, instead of requiring every drive
already within the 0.02 rad open-classification band. No production guard was
changed or bypassed. Hardware-free harness checks passed.

Attempt 03, 08:59:18–08:59:36 UTC, passed preparation. Startup opened only 111
from -0.026806 rad; 113 and 447 stayed open. Then 447 and 111 closed in that
order and each held three seconds. All motors were explicitly disabled,
GRIP shut down cleanly and its state was saved with `clean_exit: true`.

| ID | Saved position rad | Saved state | Final disabled readback rad |
| --- | ---: | --- | ---: |
| 113 | 0.004794 | open | 0.004679 |
| 447 | 0.665978 | partial | 0.665748 |
| 111 | 0.689563 | partial | 0.687760 |

The jaws relaxed after removing holding torque; partial is the measured state,
not an assertion that the 0.75 rad closed endpoint was reached. Final readback
confirmed state 64, mode 0, watchdog 100 ms, near-zero torque and no hardware or
communication faults for all three. The isolated containers/network were removed.

Live persistent checkpoint:
`/tmp/candle-grip-integration.VrnUwa/state/grippers.json`.
The preparation output contains an independent `saved_after_shutdown.json`
copy, preserving the pre-power-cycle reference even if startup later updates
the live checkpoint. Archived attempts, telemetry, logs, saved snapshots and
harness are at
`/home/grip-pc/Documents/candle-validation-2026-09-18.h5mqSv/cold-prepare-01-03.tar.gz`.

## Power-off check

The operator reported motor power off. At 09:03:57 UTC the USB adapter remained
enumerated as bus 001/device 005, VID:PID 0069:1000; no driver was running.
The existing read-only SDK probe failed its first read, for ID 111, with CAN
transfer failure (exit 1, not the outer timeout). This is consistent with motor
power being off; the probe stops on its first failure, so it did not independently
check 113/447 while off. No enable/disable/zero or motor register writes were issued.

The live persistent state compared byte-for-byte equal to the preparation
snapshot. SHA-256:
`0ee0293288a193fa1a925fffde0ca278e821797ec673fadb00fd6eee6b826754`.
Readback output is saved alongside the archives as `cold-power-off-readback.txt`.

## Power-on result: reference mismatch, no recovery motion

After the operator confirmed power on, the exclusive SDK read at 09:05:43 UTC
returned all three motors disabled (state 64, mode 0), watchdog 100 ms,
hardware/communication/motion status zero and near-zero torque.

| ID | Saved rad | Power-on rad | Difference rad |
| --- | ---: | ---: | ---: |
| 113 | 0.004794 | -0.156850 | -0.161644 |
| 447 | 0.665978 | -0.042530 | -0.708507 |
| 111 | 0.689563 | -0.101626 | -0.791189 |

Every difference exceeds the unchanged 0.05 rad saved/live reference tolerance.
111 and 113 also report outside the [-0.05, 0.75] rad calibrated travel envelope.
The live saved-state file remained byte-for-byte identical to the preparation
snapshot, with the same SHA-256 recorded above. The problem is not loss of the
saved JSON; the live position reference is not compatible with it after cold boot.
The exact firmware zero/turn-reset mechanism has not yet been established.

The supervisor did not start GRIP or issue any enable, zero, motion or motor
register write after power restoration. The installed StateStore reference
check was separately replayed against all three recorded readings in a
network-isolated container without USB or ROS initialization; it rejected each
one, as required. This is a hardware-free check of the real guard, **not** a claim
that a complete live cold-start initializer/recovery sequence passed.

Readback and guard replay output are saved in the same archive directory as
`cold-power-on-readback.txt` and `cold-reference-guard-check.txt`. All one-shot
containers exited; no motor driver is running. The saved mixed state remains
unchanged for diagnosis. Do not launch the normal pipeline expecting automatic
opening until a valid reference has been restored and verified.

## Next work

Investigate a trustworthy cold-boot calibration/reference mechanism before
unattended next-day deployment, then repeat this physical test. Do not guess a
turn offset, overwrite the historical checkpoint or re-zero at an unknown jaw
position. Successful recovery must open only 447 and 111, leave 113 open, replay
no close or holding-torque command, and finish disabled with a saved open snapshot.

## Root-cause investigation after the requested fix

Read-only register diagnostics confirmed all three drives have `motorGearRatio`
0.1 (10:1 reduction), `auxEncoder` 0, `auxEncoderMode` 0, `homingMode` 0,
`reverseDirection` 0 and zero main/aux encoder error status. All were disabled.
The configuration readback is archived as `cold-encoder-configuration.json`.
An oversized diagnostic batch was rejected by the SDK; a subsequent batch with
unsupported metadata also failed. Individual reads of the fields reported here
succeeded. No hardware configuration, calibration or motor command was written.

The local SDK's `MD::zero()` writes only `runZero=1`; `MD::save()` is a separate
command. Existing driver explicit zero calls do not call save. MAB's documented
zero procedure requires save for power-down persistence. In addition, a rotor
encoder behind a gearbox cannot uniquely identify the output shaft position at
startup; an auxiliary output encoder is the native hardware solution. Sources:
[MAB zero/save](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/candlelib/software_package/legacy/MDTOOL.html),
[MAB encoders](https://mabrobotics.github.io/MD80-x-CANdle-Documentation/MD/encoders.html).

These are two distinct requirements: a persistent calibrated zero and a valid
turn-count reference. Saving a zero alone cannot preserve accumulated turns.
The 0.8 rad permitted travel spans more than the approximately 0.628319 rad
output-angle period of one rotor turn. Saved-state recovery therefore needs an
explicit operational guarantee against unobserved movement while powered off;
whole-period movement cannot be rejected using rotor feedback alone. Without
that guarantee, recovery must establish a known position by homing or use an
output encoder. The operator has been asked which contract applies before an
automatic restoration path is implemented. No production changes yet.

## Subsequent decision and implementation

The user selected opening-stop homing (2 rad/s, 0.5 Nm) and removed the saved-state
requirement. Startup and motor-power recovery now use homing rather than restored
positions. See [homing validation](2026-09-18-homing.md) for the implementation,
successful startup hardware checks, and remaining runtime power-cycle check.
