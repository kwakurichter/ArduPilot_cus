# Offline cross-anchor geometry evaluation

AI-assisted diagnostic tooling. This does not reject ranges, change firmware, or
feed the EKF. It investigates whether four ranges can distinguish obstruction
from real movement without assuming a fixed tag height.

From the repository root, with numpy, matplotlib and pymavlink installed and a
Clang C++ compiler available:

```sh
MPLBACKEND=Agg python3 -m unittest discover -s libraries/AP_Ranging/tools/range_geometry_eval -v
MPLBACKEND=Agg python3 libraries/AP_Ranging/tools/range_geometry_eval/replay.py \
    --output /tmp/range-geometry LOG63.bin LOG65.bin LOG69.bin
```

The tool compiles the unchanged checked-out production solver with ASan/UBSan
and warnings as errors. Its binary reader checks survey and configuration
consistency. Outputs include per-log JSON/PNG and a source/input SHA-256 manifest.
Exactly four surveyed anchors are required for this experiment.

## Method and limits

- Use each log's own RANC survey and RNG_A_NSE. Reject intra-log survey or ranging
  configuration changes; never silently reuse another recording's anchor map.
- Sample a causal 200 ms grid: only delivered packets are eligible. Use acquisition
  timestamps, monotonic updates within a session, and clear an anchor's old sample
  when its session changes. Require four ranges less than 300 ms old, with no
  more than 150 ms spread. Inflate solver noise by the existing spread allowance.
- Replaying the production mathematical solver is not a replay of the complete
  AP scheduler, transport, beacon backend or EKF. Log delivery timing approximates
  availability. The grid is 5 Hz; percentages are grid occupancy, not packet rates.
- Enumerate both exact three-sphere intersections after omitting each anchor.
  No position seed, preferred height branch, smoothing, noise inflation or tag
  ground-truth position chooses a hypothesis. Impossible sphere intersections
  produce no candidate. This is a diagnostic, not a robust noisy estimator.
- Count anchors for which at least one candidate implies a positive held-out
  range excess over 0.10 m. This threshold illustrates ambiguity; it is not a
  proposed rejection threshold. Multiple candidates do not identify a bad anchor.
- Pairwise triangle bounds are necessary but not sufficient. Strict violations
  can also result from noise or measurements acquired at different tag positions.
- Signed plane distance uses a best-fit anchor plane with a normal towards
  positive Down: positive means below the plane, negative above. A negative
  value is not intrinsically invalid in flight; the stationary bench description
  establishes the unexpected branch in these recordings.

## Recorded evidence, 2026-10-19

Logs 63 and 65 used a different survey from log 69. Their centered-map singular
values were approximately 1.879, 1.746, 0.00139 m (almost coplanar); log 69 used
1.879, 1.744, 0.0774 m. The unchanged production solver accepted none of the
1,593/1,261 eligible grid points in logs 63/65. This limits what those recordings
can validate about the complete position path; it does not prevent examining
range geometry.

Log 69, using approximate phase windows relative to first RTWR delivery:

| Window | Eligible / accepted | Above plane among accepted | Median fit RMS | Median Down |
| --- | --- | --- | --- | --- |
| Clear, 20–110 s | 445 / 445 | 27.0% | 1.18 cm | -0.051 m |
| Early blockage, 130–150 s | 100 / 100 | 100% | 5.18 cm | -1.773 m |
| Middle blockage, 160–200 s | 200 / 200 | 87.5% | 1.00 cm | -1.693 m |
| Late blockage, 220–235 s | 75 / 75 | 100% | 3.54 cm | -1.749 m |
| Final clear, 250–300 s | 250 / 250 | 16.0% | 0.96 cm | -0.052 m |

The medians hide two height branches. A small RMS residual is not evidence that
height is correct: in the middle blocked window it is as small as in clear data.
BCN_TYPE was zero in all three recordings, so these are offline candidate
positions, not observed EKF jumps.

During 130–235 s blockage, pairwise bounds detected none of 525 eligible sets in
log 65 or 523 in log 69. Multiple positive omitted-anchor hypotheses occurred in
520/525 and 494/523 respectively. Log 69 already had multiple hypotheses in
309/445 clear sets. A four-range leave-one-out selector is therefore not justified.
In log 63's approximate 120–250 s movement interval, strict pairwise bounds failed
on 24/647 sets (up to 0.179 m). These are not verified false positives: the true
trajectory, exact sample simultaneity, and obstruction-free status are unknown.

## Continuing with the current four anchors

The user selected four-anchor operation. An additional offline restriction to
positions below the best-fit anchor plane was therefore tested, using the known
bench placement only. It still left multiple positive-error anchor hypotheses
in 518/525 blocked sets in log 65 and 479/523 in log 69 (130–235 s). In neither
blocked window did it uniquely identify anchor 8 under the illustrative 10 cm
excess rule. Restricting the side removes the mirror ambiguity but does not
solve fault identification. It must not be assumed valid for every flight.

Do not tune a live rejecting filter to these three recordings. For the current
four-anchor system, the next design should explicitly separate branch selection,
obstruction evidence, and availability. Evaluate a validated independent height
constraint or a declared operating-side restriction in shadow mode. A barometer
or ToF measurement needs a valid transformation to the surveyed anchor frame;
assuming Down=0 because the tag sat on a bench is not a flight implementation.
Temporal continuity can stabilize a branch but cannot by itself distinguish a
slow range bias from real motion. When several hypotheses remain plausible,
flag the position as unreliable instead of arbitrarily excluding an anchor.

Withholding one of four ranges leaves three ranges: unconstrained 3D positioning
is ambiguous, so a validated extra constraint or an unavailable output is needed.
These recordings do not yet justify a production rejection policy. Solver and
EKF changes require separate review and tests. The synthetic fifth-anchor test
is retained only as a mathematical illustration of redundancy, not a requirement
to add hardware or a universal guarantee.
