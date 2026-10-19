# ArduSwarm four-message ranging

This implementation extends AP_Ranging with peer TWR and surveyed-anchor TWR.
AP_Beacon adapts anchor observations to the existing EKF3 beacon path. There is
no new general positioning library and no TDoA mode in this version.

**Development firmware:** simulation and build results are recorded in
[VALIDATION.md](VALIDATION.md). Hardware timestamp, calibration, radio-load,
stack, and estimator review gates remain before use for navigation on a vehicle.
The implementation and documentation were AI-assisted.

## Compatibility

The radio protocol is ArduSwarm TWR version 1. It replaces the old three-message
AP_Ranging peer protocol and is incompatible with stock Bitcraze TWR frames.
Upgrade all participating peer vehicles together. Every peer must include the
other vehicle in its `RNG_PEER_n` roster. Both endpoints obtain a sample from one
successful exchange; a reverse poll is not necessary. Peer `TWR` logs and the
existing companion TUNNEL v1 layout remain unchanged. Anchor observations are
not put into that peer snapshot.

All radios must use matching network/channel/profile and compatible timing.
`RNG_NET_ID` separates networks; it is not authentication. Roll back by restoring
a consistent previous firmware set on every peer. Preserve parameter backups;
existing parameter indices are unchanged. Mode/RF/timing/identity/roster/map
changes take effect on reboot. Pending changes invalidate anchor navigation.

## Vehicle configuration

- `MAV_SYSID`: tag identity, 1–255. Tag 1 and anchor 1 are different identities.
- `RNG_TYPE=1`: DW1000 hardware; `10` is a SITL-only range generator.
- `RNG_MODE=0`: peer mode (default); use `RNG_PEER_1` through `RNG_PEER_4`.
- `RNG_MODE=1`: anchor mode; peer ranging is inactive in this mode.
- `RNG_NET_ID=1`, `RNG_CHAN=2`, `RNG_REPLY_US=3000`: initial matching settings.
- `RNG_ANT_DLY=16384`: starting calibration value, not a calibrated result.
- `RNG_A_POLL_MS=10`, `RNG_A_TMO_MS=30`: anchor scheduling defaults.
  Peer equivalents are `RNG_POLL_MS=50`, `RNG_XCHG_MS=30`.
- Enable each surveyed physical ID with `RNG_A1_EN` … `RNG_A8_EN`.
  Set its `RNG_An_POS_X`, `RNG_An_POS_Y`, `RNG_An_POS_Z` in meters, north/east/down
  from the beacon origin. A point two meters above the origin has Z=-2.
  `(0,0,0)` is valid. EN=0 explicitly leaves a slot empty.

Only enabled IDs are polled. Sparse IDs such as 1/3/6/8 work. A failed target is
retried with bounded 100–800 ms backoff while other targets remain eligible.
Exchanges have a whole-transaction deadline. The configured timeout must cover
four delayed transmissions plus at least 3 ms margin. Timing values are cached
at boot, including the reply delay. The radio SPI callback runs every 300 us;
its real worst-case latency under shared SPI load must be measured.

Allow one to three anchors for range/calibration diagnostics with `BCN_TYPE=0`.
For positioning use `BCN_TYPE=4`, a consistent `BCN_LATITUDE`, `BCN_LONGITUDE`,
`BCN_ALT` origin, and `BCN_ORIENT_YAW=0`. Reboot after setting the survey and origin.
Keep the tag antenna offset from the EKF reference negligible within the measured
error budget; this version has no attitude-dependent antenna lever-arm correction.
Four spatially distinct, non-coplanar anchors are the minimum for this full 3D
implementation; four at the same height are rejected even with barometric height.
Eight slots do not guarantee useful geometry. Duplicate positions closer than
5 cm are rejected.

The EKF3 configuration exercised in SITL uses `EK3_SRC1_POSXY=4`,
`EK3_SRC1_VELXY=0`, `EK3_SRC1_VELZ=0`, `EK3_SRC1_POSZ=1` (barometer) or
`4` (beacons), and an independently valid yaw source. Ranges do not provide yaw.
Set `EK3_BCN_DELAY=0` initially because the adapter preserves acquisition time;
calibrate any remaining hardware delay separately. `RNG_A_NSE=0.3` controls the
bootstrap solver's range noise, while `EK3_BCN_M_NSE` controls EKF range noise.
Neither value is a measured accuracy claim. The autotest disables GPS explicitly;
copy its source choices only after reviewing the intended vehicle configuration.

Selecting `BCN_TYPE=4` requires a healthy anchor solution at arming, including when
another EKF source is selected. Use BCN_TYPE=0 for range-only bench operation.
This conservative gate also reports pending reboot changes.

## Data and estimator behavior

POLL → RESPONSE → FINAL → REPORT is shared by both modes and the node firmware.
FINAL contains the initiator's actual POLL TX and RESPONSE RX timestamps and its
hardware-scheduled FINAL TX timestamp. The responder calculates a range and
returns its three timestamps in REPORT. The initiator calculates independently.
A lost REPORT leaves a new responder sample and no new initiator sample.
Actual TX timestamps must equal the programmed delayed-TX timestamps. This
requirement, especially FINAL's advertised timestamp, still needs bench proof.

The transport uses explicit little-endian bytes, 40-bit device timestamps,
role/ID pairs, network, sequence, and initiator boot-session correlation. No C
struct is serialized. The common header is copied byte-for-byte to node firmware;
the portable test runner checks both copies and their test fixtures.

The driver owns radio state on the SPI callback and passes completed samples
through a bounded queue to the main loop. Queue overflow drops observations.
The range timestamp approximates the exchange midpoint in the local millisecond
clock and removes REPORT time; host scheduling latency still needs measurement.
Peer reverse polls are suppressed briefly after either endpoint role succeeds.

The anchor adapter maps enabled physical IDs to stable dense beacon instances.
It preserves per-anchor sample times and invalidates all navigation aiding if
fewer than four ranges younger than 300 ms remain, their timestamp spread exceeds
150 ms, or geometry/solver checks fail. Stale observations never receive fresh
timestamps. Individual raw ranges remain available in diagnostics during outages.

A bounded single-precision multilateration solver generates the independent
bootstrap position from UWB ranges and survey coordinates only. It uses a linear
seed, at most ten Gauss–Newton steps, scaled pivot rejection at 1e-4, residual
limits of three assumed standard deviations, and a position uncertainty limit
below 1 m. Noise is inflated for timestamp spread using a provisional 3 m/s motion
allowance; this is not a proven flight speed limit or NLOS rejection guarantee.
The solver does not use EKF position or simulator truth as its seed. Truth is
used only by the SITL backend to generate ranges. The EKF still performs its own
alignment and fuses individual ranges; the solved position is not fused again as
an additional independent measurement.

EKF3 beacon arrays now support eight instances, DAL refreshes the instance count,
and replay rejects out-of-range instance/count values. Successful beacon fusion
clears position/velocity timeout flags after a dropout; the regression test
exercises this previously latched recovery path. EKF2's existing ten-element
array was inspected, but this release's navigation validation targets EKF3.

## Diagnostics and tests

At startup, a selected DW1000 backend reports `UWB: DW1000 initializing`, then
`UWB: init OK` with the mode, system ID and target count, or `UWB: DW1000 init
FAILED` alongside a specific error. Success requires chip identification,
configuration and callback registration; it does not mean ranges or a navigation
solution are available. Callback registration failure leaves the radio idle.
Allocation failure is reported separately. With `RNG_DEBUG=1`, selecting
`RNG_TYPE=0` also reports `UWB: disabled (RNG_TYPE=0)` at startup.

On Crazyflie, SD, Flow and UWB share SPI1. The UWB callback period remains 300 us
and runs with the shared bus semaphore held. Setting `RNG_TYPE=0` and rebooting
creates no UWB backend and registers no UWB callback, but does not physically
isolate the attached deck. An SD failure that persists in that configuration
needs investigation of the deck connections, pin assignments, startup and signal
integrity before attributing it to ranging traffic. `Failed to create log
directory` can also indicate a later filesystem I/O failure; record its complete
error text and whether the earlier SD mount succeeded. In this FATFS backend,
`ENOSPC` from `mkdir` maps to `FR_NOT_ENABLED` (no filesystem object registered),
so it is not evidence that the SD card is full. This identifies a startup/mount
problem but does not by itself identify the hardware or software cause.

The current board definition expects SD CS on IO4/PC12, Flow CS on IO3/PB4,
UWB CS on IO1/PB8, UWB IRQ on RX1/PC11 and UWB reset on TX1/PC10. Modified deck
solder bridges must match this mapping. Bitcraze documents alternate SD SPI and
Loco IRQ/reset wiring in its [SD deck documentation](https://www.bitcraze.io/products/micro-sd-card-deck/).
Do not change the board mapping without checking the actual deck wiring.

- `RTWR`: completed hardware exchanges, acquisition time, session, sequence,
  remote ID, initiator ID, local role, and range.
- `RANC`: physical anchor ID, enabled/fresh state, age, sample counter, range,
  and survey position. No sample has age UINT32_MAX.
- `BCN`: existing aggregate beacon health/position log; its four range columns
  remain unchanged. Use RANC for all eight physical anchors.
- `RNG_DEBUG=1`: `RWDG` diagnostic snapshots logged approximately once per
  second, even with no ranges.
  Snapshots cross from the SPI thread through a bounded, nonblocking handoff;
  only the main loop writes logs. `Fail` counts aborts, `TxSt` unexpected TX state,
  `TxDif` an actual/scheduled TX timestamp mismatch, `Exp` transaction expiry,
  `RxErr`/`RxTO` hardware receive errors/timeouts, `SPI` failed HAL transfers, and
  `QDrop` range delivery drops. Counters are cumulative. `Gap` is the largest
  callback-start interval since the previous snapshot; `GapMax` and `Work` are
  lifetime maximum callback-start interval and callback-body duration, in us.
  Gaps include bus-lock/scheduling delay; they do not identify which SPI client
  held the bus. Work excludes the snapshot copy at the end of the callback.
- `RWTX`: latest TX rejection's event time, remote ID, pre-abort protocol state,
  sequence, expected/actual 40-bit DW1000 timestamp (ticks), and callback gap
  (us). Several events within one snapshot interval coalesce to the latest;
  `RWDG` retains their cumulative counts. It is not a complete event trace.
  State values follow `twr_state` in `twr_protocol.h` (0 is idle).
- `RNG_SIM_MASK`: SITL operational-anchor bitmask; it changes availability without
  changing the survey. Hardware builds do not expose it.

From the repository root:

```sh
python3 libraries/AP_Ranging/tests/run_tests.py --node /path/to/cus-node-firmware
./waf configure --board sitl --no-submodule-update
./waf copter
./waf replay
Tools/autotest/autotest.py test.Copter.AnchorRanging test.Copter.BeaconPosition
./waf configure --board crazyflie2 --extra-hwdef libraries/AP_Ranging/anchor-beacon.hwdef
./waf copter
```

Run the Copter and Replay builds as separate Waf invocations; the combined
`./waf copter replay` command encountered a common-library linker failure during
validation. See [VALIDATION.md](VALIDATION.md) for the results.

Use `crazyflie2_bl` for the bootloader variant. The extra hardware definition
selects only the ranging beacon backend. The normal board defaults retain peer
ranging without enabling beacon navigation. Physical throughput/fairness with
multiple tags, antenna calibration, CPU/stack use, and controlled flight remain
separate acceptance gates in [ANCHOR_TWR_PLAN.md](ANCHOR_TWR_PLAN.md).

## Four-anchor bench acceptance

Keep the vehicle disarmed, `BCN_TYPE=0`, and its current EKF source selection
unchanged for the transport tests. Keep the SD, Flow and UWB decks fitted.
Calibration is separate: retain each radio's current antenna-delay setting.
The tag value 16557 used in the October bench runs is specific to that tested
pair and is not a new default or a calibration for additional anchors.

Use physical anchor IDs 1, 3, 6 and 8 to exercise empty slots in the middle of
the roster. Configure each node with its unique ID, mode 5 (`w`), network 1,
channel 2, reply 3000 us and timeout 30 ms. Reboot and save `@status` output
from each node to verify active settings and USB/console persistence.
Keep the tag's MAV_SYSID at 1; tag 1 and anchor 1 have different identity roles.

On the tag set `RNG_TYPE=1`, `RNG_MODE=1`, `RNG_NET_ID=1`, `RNG_CHAN=2`,
`RNG_REPLY_US=3000`, `RNG_A_POLL_MS=10`, `RNG_A_TMO_MS=30`, `RNG_DEBUG=1`,
and `LOG_DISARMED=1`. Enable only anchors 1/3/6/8, disabling 2/4/5/7. Enter
finite, distinct approximate XYZ positions for the four enabled anchors before
rebooting: the current frontend rejects duplicate positions within 5 cm even
with `BCN_TYPE=0`. Exact survey accuracy and non-coplanar geometry are required
for the later positioning test; approximate coordinates here are only for
range-only diagnostics and must not be used to enable navigation.

Record one continuous tag log, keeping the tag stationary and noting the event
times relative to boot:

1. All four anchors powered for 60 seconds.
2. Power anchor 6 off for 15 seconds, then on for 30 seconds.
3. Power all anchors off for 15 seconds, then on for 30 seconds.
4. Stop and save the log; collect each node's `@status` output again.

Inspect fresh `RTWR` samples by Remote ID, and `RANC` by physical ID. Each enabled
anchor should update without starvation. Disabled slots must have no completed
exchanges. During the single-anchor outage its sample counter must stop and
its age increase until Ok becomes 0 (300 ms freshness threshold); the other
three must continue. Recovery must come from new samples without rebooting the
tag. The all-anchor outage must invalidate all four and then recover. Missing
range samples during complete silence is expected; RWDG logging should continue
while callbacks and the main loop are serviced. A retained Dist with Ok=0 is stale,
not a new measurement.

Measure per-anchor rate, inter-sample gaps, sweep timestamp spread, and recovery
time. For later positioning, four fresh observations must fit the 300 ms age
and 150 ms timestamp-spread limits; do not relax the limits to conceal dropouts.
Check DSF logger drops, TWR queue/timestamp failures, PM internal errors/free
memory and STAK free stack under the shared-SPI load. No fixed per-anchor rate
is promised before this physical test.

Subsequent bench tests are tag reboot/rejoin, an absent enabled target, a node
on a different network (restore and reboot afterward), and correlated node/tag
ranges matched by tag ID, session and sequence. Then survey four non-coplanar
anchors and validate the bootstrap position, stationary EKF behavior, and
fewer-than-four-anchor rejection/recovery. Peer mode and multiple-tag contention
require additional tag hardware. Controlled flight remains a separate gate.


## Retired bench experiments

The SD busy-release experiment was withdrawn after a logging deadlock. The
512-byte logger-write profile and SPI/SD timing instrumentation have also been
removed. Normal SD transaction handling and 4096-byte logger writes remain,
including the working deck coexistence fixes.

The temporal range-gate shadow experiment did not reject sustained blockage bias
and has been removed, along with its build profile and RFLT logging. Parameter
index 28 remains reserved: old RNG_F_SHDW values are ignored, and no existing
parameter is renumbered. Firmware still publishes validated raw TWR ranges;
no replacement outlier filter is enabled. RWDG/RWTX logs remain available with
RNG_DEBUG; the recurring five-second counter message has been removed.

### Per-anchor radio diagnostics

Build ArduPilot with `--extra-hwdef libraries/AP_Ranging/radio-diag.hwdef`.
The optional feature `AP_RANGING_RF_DIAGNOSTICS_ENABLED` defaults off; the bench
profile includes the anchor backend and radio diagnostics only. `RNG_RF_DIAG` is a reboot-
required anchor-ID bitmask (new index 29), default 0. Set 128 to select anchor 8.
Only selected anchors need the matching updated node firmware. **An older node
cannot answer the extended diagnostic POLL**; disable the bit and reboot to
restore ordinary exchanges if its firmware has not been updated. Peer mode does
not request diagnostics. Node `@status` advertises `radio diagnostics supported=1`.

The node accepts ordinary polls and returns ordinary reports unless requested.
No EEPROM layout, antenna delay, RF profile, ranging equation, SD behavior or
positioning rejection policy changes. The diagnostic node remains compatible
with previous tags when they use ordinary exchanges. The extension leaves the
v1 core codec unchanged: a POLL may append marker 0xD1 (17 bytes); a requested
REPORT appends 0xD1 and two 12-byte reception snapshots (57 bytes total).
All other frame sizes and markers are rejected. Each snapshot carries a 40-bit
uncorrected receive timestamp, signed centi-dBm receive/first-path power,
signed correction ticks and validity flags. Endianness is explicitly little-endian.

For each completed selected-anchor exchange, the tag writes three `RFRX`
records and one `RFRG` record to the normal BIN. No separate node USB capture
is required. Session, Seq and Anchor identify the exchange; do not join on Seq
alone because it wraps. Raw RTWR records remain available.

| RFRX field | Meaning |
| --- | --- |
| TimeUS | Tag logging time; not the anchor's capture time |
| Session, Seq, Anchor | Initiator session, exchange sequence, anchor ID |
| Pkt | 1=POLL received at anchor; 2=RESPONSE received at tag; 3=FINAL received at anchor |
| RxP, FpP | Signed centi-dBm (divide by 100); -32768 invalid |
| Corr | Signed DW1000 ticks: corrected timestamp minus Raw, modulo 40 bits |
| Raw | Uncorrected receive timestamp in the receiving radio's own clock domain |
| Flags | Bit 0: RxP valid; bit 1: FpP valid; bit 2: correction representable; 7 means all valid |
| CapUS | Tag RESPONSE capture duration in microseconds, including normal timestamp reads/correction; 0 for remote snapshots, whose duration is not measured |

RFRG contains `TimeUS,AcqMS,Session,Seq,Anchor,Dist,Uncorr,Valid`. Dist is the
normal corrected range. Uncorr is a diagnostic recalculation from the same
three transmit timestamps and three uncorrected receive timestamps. It removes
only libdw1000's software received-power correction; hardware antenna delays
and receiver timestamp estimation remain. Valid is 1 only when all three
snapshots are valid, their raw-plus-correction timestamps exactly match the
engine's same-exchange corrected timestamps, and the recalculation passes the
existing numerical bounds. Otherwise Uncorr is NaN and Valid=0. Never substitute
Uncorr into navigation based on this experiment alone.

Capture occurs before the received packet's registers can be replaced by the
next reception. The existing corrected timestamp path is preserved by calling
its raw-read and correction functions directly. Six additional SPI reads per
selected packet obtain power diagnostics, with no libdw1000 submodule edits.
Only one of these captures per exchange uses the tag's shared SD/flow bus;
the anchor captures POLL and FINAL on its own bus. Diagnostics add 145 log bytes
per selected exchange (about 2.4 KB/s at 16 exchanges/s, excluding metadata),
in addition to normal ranging logs. Radio extensions add 26 payload bytes per
exchange. The queue remains bounded; increased work must be checked on hardware.

Bench procedure:

1. Flash `node-radio-diag.dfu` onto anchor 8. Preserve its existing ID 8, mode 5,
   network/channel and antenna settings; confirm them with `@status` after reboot.
2. Flash `arducopter-radio-diag-with-bl.hex` onto the tag. Set `RNG_RF_DIAG=128`,
   set `RNG_DEBUG=1`, `LOG_DISARMED=1`, `BCN_TYPE=0`, and reboot.
   Confirm `UWB: radio diagnostics mask=128`. Other anchors may keep their current
   firmware. Do not change calibration between comparison recordings.
3. Repeat two minutes clear, two minutes with your stationary body blocking
   anchor 8, then at least 30 seconds clear. Keep the tag and antennas fixed and
   note the blockage/removal times. Remain disarmed.
4. Share the BIN. Check for RFRX Pkt 1/2/3, RFRG Valid=1, matched radio quality and
   correction changes, and compare Dist versus Uncorr through the transition.
   Also check RWDG, DSF drops and capture duration for added timing cost.
5. `RNG_RF_DIAG=0` followed by reboot restores ordinary packet sizes and disables
   the additional captures/log records.

### Offline cross-anchor geometry checks

`tools/range_geometry_eval/README.md` documents the logs 63/65/69 investigation,
replay commands and limitations. Four-anchor leave-one-out fits did not uniquely
identify the blocked anchor. The unchanged solver could accept the opposite
height branch even during clear portions of log 69. Keep beacon navigation
disabled for this bench investigation; no live rejection is implemented by the
offline tool. Additional geometric redundancy or a validated independent height
constraint needs evaluation before adopting a position or rejection policy.
