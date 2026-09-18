# Blind bottle/foil regression — 2026-09-18

Requested order: 113 → 447 → 111; each close, hold 3 seconds, then open.
Object identities were not requested. No gain, limit or zero changes.

## Attempt 01: preflight stopped, no commanded motion

At 08:28:49 UTC, exclusive SDK readback showed all drives disabled (state 64,
mode 0, watchdog 100 ms), hardware/communication status 0 and near-zero torque:

| ID | Position rad | Motor °C |
| --- | ---: | ---: |
| 113 | 0.004487 | 28.50 |
| 447 | -0.007248 | 28.75 |
| 111 | 0.609182 | 28.20 |

111 was outside the test's 0.02 rad initial-open band. The harness stopped
before launching the driver, enabling a motor or issuing any motion request.
Physical position/reference must be reconciled before retrying; this is not
a completed holding test or a demonstrated driver fault. Final readback again
confirmed all three disabled. The isolated test containers/network were removed.

The final braking-fixed driver binary was verified as SHA-256
`260fcb6b1a497383a5e34e69ac820841b4ce3d781f4f2e3e2a7e537058c6d639`.
The existing guarded harness was reused and its hardware-free checks passed.

Raw result, telemetry header, harness and Compose override are archived at
`/home/grip-pc/Documents/candle-validation-2026-09-18.h5mqSv/bottle-foil-01-preflight.tar.gz`.

## Attempt 02: sequence and observed retention passed

After the operator corrected the placement and renewed clearance, preflight
read 111 at 0.006750 rad, 113 at 0.004372 rad and 447 at -0.007286 rad.
No reference, gain or limit was changed. The exact braking-fixed binary above
completed the real GRIP launch/Zenoh SoftCloseGripper/open service path from
08:34:02 to 08:34:32 UTC, in order **113 → 447 → 111**. Each closed, held for
three seconds and opened before the next gripper closed. Startup issued no
opening or closing commands. The operator subsequently confirmed that both
the bottle and foil stayed captured throughout their holds; object identities
were not supplied and were not used by the test.

Last 0.5-second windows of the hold and released opening:

| ID | Hold position rad | Hold torque Nm | Open position rad | Open torque Nm |
| --- | ---: | ---: | ---: | ---: |
| 113 | 0.733395 | 0.744924 | 0.004659 | -0.000084 |
| 447 | 0.482267 | 1.999840 | 0.005006 | -0.000332 |
| 111 | 0.733208 | 0.706943 | 0.000067 | -0.000075 |

447 entered object torque hold at 2 Nm. 113 and 111 recovered from their
near-endpoint contact transitions, then held the torque captured at the closing
timeout. Neither confirmed the 0.75 rad closed endpoint: this run therefore
validates observed retention with the final braking change, not a physically
settled 0.25 Nm closed-endpoint mode.

All three openings used the configured two-second timeout release: PD disabled,
commanded torque zero, measured torque near zero. They finished within the
0.02 rad startup-open classification band. Tight 0.005 rad opening accuracy is
not a general pass: 447's mean was 0.005006 rad, and changes from initial position
were +0.000156 / +0.012246 / -0.006746 rad for 113 / 447 / 111 respectively.

No health/safety guard tripped. Sampled travel stayed in [-0.05, 0.75] rad;
maximum measured absolute torque was 2.028700 Nm against the unchanged 2 Nm
command cap and 2.1 Nm measurement guard. Maximum sampled closing speeds were
2.332 / 2.256 / 2.442 rad/s, below the 4 rad/s test guard. Peak motor temperature
was 32.28 °C; peak MOSFET temperature was 39.96 °C.

Graceful shutdown passed. Final exclusive hardware readback confirmed all three
disabled (state 64, mode 0, watchdog 100 ms), hardware/communication status zero,
near-zero torque, positions 113=0.004794, 447=0.005100, 111=0.000077 rad.
Only the isolated test containers and network were removed; raw records remain.

Raw telemetry, launch log, result, harness and Compose override are archived at
`/home/grip-pc/Documents/candle-validation-2026-09-18.h5mqSv/bottle-foil-02-passed.tar.gz`.
The raw machine result's visual-confirmation field remains null because the
operator's confirmation arrived afterward and is recorded here.

This used the final local driver/GRIP overlays on the existing container image;
it did not replace the production image, update its dependency pin, commit or
push. Physical power-off/on recovery remains untested.
