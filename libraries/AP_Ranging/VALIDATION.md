# Validation and remaining acceptance work

This implementation and its tests/documentation are AI-assisted. This is
**development firmware**, not a claim of flight readiness. Calibration remains
paused. The tested tag's provisional antenna delay (16557) is not a firmware
default or a fleet-wide calibration.

## Current implementation

- Shared four-message peer/anchor TWR protocol, bounded transaction deadlines,
  timestamp validation, sparse anchor IDs and retained acquisition times.
- Surveyed anchor parameters, optional AP_Beacon/EKF3 integration, configuration
  change detection, freshness checks and pre-arm validation.
- Working Crazyflie SD/Flow/UWB coexistence fixes and deck LEDs are preserved.
- Node USB PMA access/receive fixes, console stack/queue capacity, EEPROM bounds
  and readback checks, and MAVLink console framing protection are preserved.
- Optional per-anchor receive-quality capture remains disabled by default.
  `radio-diag.hwdef` enables only the anchor backend and radio diagnostics.
- RWDG/RWTX logs remain available with RNG_DEBUG. Startup success/failure messages
  remain; the recurring five-second TWR counter message has been removed.

## Retired experiments

The SD busy-release experiment caused a hardware logging deadlock and was
withdrawn. The 512-byte write profile, SPI/SD timing collectors, and their tests
and build profiles have now been removed. The SD/SPI runtime matches the working
baseline, including R1 start-bit handling and release clocks; normal 4096-byte
logger writes remain. No ChibiOS submodule edits are included.

The temporal gate's firmware shadow mode and offline evaluator were removed
because they did not solve gradual obstruction bias. Tests specific to that
retired implementation were removed with it; transport, radio, solver, EEPROM,
USB and retained geometry tests remain. The reusable binary reader and solver
harness now live with the geometry evaluator, which has no filter dependency.
Parameter index 28 is reserved for retired RNG_F_SHDW; no parameter is renumbered.

The complete pre-cleanup changes, including experimental source and the old
chronological validation record, are archived in the local commit-preparation
artifacts. Removed profiles are not compatibility aliases: use the current
anchor or radio diagnostic profile explicitly.

## Checks repeated during cleanup, 2026-10-19

- Portable Clang suite passes with warnings as errors and ASan/UBSan. It covers
  protocol framing/dropout/session/sequence/wrap, two-sided range calculation,
  stale DW1000 interrupt recovery, production multilateration, console framing,
  EEPROM persistence/bounds/failure handling, and both copies of the diagnostic
  codec and actual driver capture. The actual node callback is exercised using
  mock radio registers. Shared headers and copied fixtures match byte-for-byte.
- Node optimized ARM USB PMA test: 1136/1136 cases pass. This checks halfword PMA
  accesses, odd-length and unaligned buffers under emulation; it does not test
  enumeration. The test requires Unicorn outside this machine's sandbox.
- Node EEPROM Unity suite: 9 tests, zero failures.
- Geometry tool: 17 Python tests pass, including clock wrap, causal delivery,
  freshness/spread boundaries, sphere intersections, mirror ambiguity and
  multiple positive-error hypotheses on a known side of the anchor plane.
- After relocating its reader/harness, the log 69 geometry replay output JSON
  matches the pre-cleanup result byte-for-byte. The production C++ solver is
  compiled unchanged with ASan/UBSan and a known-position smoke fixture.
- Dedicated node clean build passes: text 87,804, data 996, BSS 12,252 bytes.
  Existing vendor/newlib and RWX linker warnings remain.
- crazyflie2_bl radio diagnostic build passes, including bootloader HEX:
  text 798,272, data 1,360, BSS 70,748; flash used 799,632 bytes.
- crazyflie2_bl standard anchor build passes with radio diagnostics disabled,
  including bootloader HEX: text 796,484, data 1,360, BSS 70,752; flash used
  797,844 bytes. Waf is left configured for this target/profile.
- AnchorRanging SITL regression passes, including sparse IDs, expiry/recovery,
  configuration-change pre-arm checks, and barometric/beacon height scenarios.
  The anchor-only profile excludes the legacy simulated beacon driver; that
  subsequent test was interrupted and rerun with the normal SITL backend set.
- The normal-backend BeaconPosition regression fails its initial one-meter
  global-position comparison (Did not get new position like old position).
  This same timing-sensitive failure was documented before cleanup, including
  reproduction on an unchanged-master baseline. It remains an unresolved test
  failure, not a passing check; the test and tolerance were not weakened.
- Python flake8 and tracked/untracked whitespace checks pass. The node's vendor
  HAL/USB files retain CRLF and are checked with core.whitespace=cr-at-eol.

Reproduce the portable checks from the ArduPilot repository root:

```sh
python3 libraries/AP_Ranging/tests/run_tests.py --node /path/to/cus-node-firmware
MPLBACKEND=Agg python3 -m unittest discover -s libraries/AP_Ranging/tools/range_geometry_eval -v
```

See README.md for hardware/SITL build and test commands. Dependencies are
pymavlink, numpy and matplotlib for offline replay, plus a host C/C++ compiler
with sanitizers. Node PMA tests additionally use arm-none-eabi-gcc, Unicorn and
pyelftools. Full SITL tests require the normal ArduPilot autotest dependencies.

## Bench evidence and known limitation

The user confirmed all three decks working together, deck LEDs, node USB and
configuration persistence after the console fix. Four-anchor logs exercised
range reporting, outages/rejoin and network separation. These establish bench
transport behavior, not position accuracy or flight acceptance. Radio log 69
contained valid same-exchange captures from both endpoints; removing software
power correction did not remove the obstruction bias.

Logs 63 and 65 used an almost coplanar survey and the current standalone solver
accepted no eligible sets in their replay. Log 69's revised survey allowed
positions, but 120/445 accepted sets in the clear 20–110 s window selected the
side above the anchor plane. During early blockage (130–150 s), 100/100 did so,
with median Down -1.773 m despite the stationary tag near the origin. In the
middle blocked window, the fit RMS was only 1.00 cm. BCN_TYPE=0 in these logs:
these are offline solver findings, not observed EKF jumps.

Restricting leave-one-out hypotheses to the known bench side still left multiple
positive-error anchor hypotheses in 518/525 blocked sets in log 65 and 479/523 in
log 69. The user selected continued operation with four anchors. Height/side
selection, obstruction confidence and invalidation need separate design and
validation; no live rejecting filter has been enabled. See
`tools/range_geometry_eval/README.md` for methodology, phase windows and limits.

## Remaining acceptance work

- Survey/height ambiguity and steady NLOS bias must be resolved before enabling
  this four-anchor configuration for navigation. A passing SITL test with good
  synthetic geometry does not address those hardware findings.
- Validate any independent height constraint against the surveyed NED frame.
  Do not turn the stationary bench assumption into a fixed-height flight rule.
- Complete per-radio calibration and endpoint timestamp/distance agreement,
  including delayed FINAL timestamp checks and actual receiver behavior.
- Measure runtime heap/stack high-water, CPU/queue loss and shared-bus latency
  under the final logging load; static BSS and compiler frame sizes are not
  runtime stack evidence.
- Test peer interoperability and one/two/four-tag contention and fairness,
  power interruptions during EEPROM writes, and supported rollback behavior.
- Review EKF integration, source/yaw/origin configuration, acquisition latency
  and dropout/recovery behavior before any controlled flight acceptance test.
- TDoA, simultaneous peer/anchor operation and stock Bitcraze wire compatibility
  remain outside this implementation.

No hardware was flashed and no commit or push was performed during cleanup.

## Local review artifacts

The pre-cleanup archive, complete post-cleanup patches/source snapshots, build
and test logs, and firmware files are in:

`/Users/kwaku/.codex/visualizations/2026/10/01/01a0f501-c16f-7ce2-9cf5-6cd4c247c9d8/ranging-commit-prep/`

The failed legacy test and interrupted incompatible-profile run are retained in
that evidence, alongside the passing anchor regression. No all-tests-pass claim
is made. Review the commit split and outstanding acceptance gates before merging
or enabling beacon navigation.
