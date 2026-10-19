# AP_Ranging peer and anchor TWR implementation plan

> Implementation status (2026-10-07): the peer/anchor protocol, node responder,
> survey parameters, beacon adapter, and EKF integration are implemented on the
> feature branches. See [README.md](README.md) for the actual configuration and
> [VALIDATION.md](VALIDATION.md) for measured results and outstanding gates.
> This plan's hardware/fleet/flight acceptance gates remain open. Implementation
> details superseding proposals include runtime bootstrap noise at index 26,
> SITL availability mask at 27, 150 ms sweep-spread gating with noise inflation,
> and a conservative arming requirement whenever BCN_TYPE=4 is selected.


Status: proposed design for implementation, 2026-10-07. AI-assisted planning.

Add a selectable fixed-anchor TWR mode to AP_Ranging and a matching ArduSwarm
anchor mode to cus-node-firmware. Tags retain MAV_SYSID identities; anchors have
fixed IDs 1 through 8 and surveyed positions. Use a four-message double-sided
TWR exchange shared by peer and anchor modes. Both participants compute their own
range from exchanged timestamps. Feed fixed-anchor ranges through AP_Beacon to
EKF3 and retain peer distances for ArduSwarm consumers. Peer ranging remains the
default mode, but its existing three-message radio protocol is replaced.

This plan defines the first implementation, validation gates, and the changes
needed in both repositories. New parameter names, packet constants, and APIs
below are proposals, not existing firmware capabilities.

## Branch and scope

- ArduPilot branch: `codex/ranging-anchor-twr`, based on local `master` at
  `5bd05d744b69d3d2ddb8d5407e95d2db2ada096b`.
- Isolated worktree: `/Users/kwaku/.codex/worktrees/ranging-anchor-twr/ardupilot`.
- Node repository: `/Users/kwaku/Documents/Crazyflie/lps/cus-node-firmware`,
  inspected on `master` at `8744c9b52521e1957f256dd494d78e5a3812a67c`.
- Create a corresponding node implementation branch when node coding starts;
  this planning task does not modify that repository.
- First release: four-message peer TWR OR anchor TWR, one shared transaction
  engine and DW1000 owner, at most eight fixed anchors, configurable subsets,
  EKF3 integration, diagnostics, and replay. Every upgraded vehicle can initiate
  and respond in peer mode; anchors are responders in anchor mode.
- Deferred: TDoA, concurrent peer and anchor ranging, moving anchors, automatic
  surveying, over-UWB configuration, deterministic swarm TDMA, and a new general
  positioning library.
- Several tags may share anchors, but supported swarm size and update rates must
  be established by contention tests. Eight anchors does not imply eight tags.

## Source findings that determine the design

1. `libraries/AP_Ranging/AP_Ranging.h` currently stores four peer distances with
   no coordinates. Parameter indices 1 and 3 are retired; existing indices
   through 13 must keep their meanings.
2. `AP_Ranging_DW1000` owns transport and the current three-message state machine.
   The responder computes the distance. The existing packets are custom raw
   frames, not the Bitcraze MAC packet format. Replace the peer exchange with
   the common four-message protocol; source findings describe the starting code,
   not compatibility requirements for the radio format.
3. The DW1000 callback is registered with `300` microseconds. Its comment says
   300 Hz, which is inconsistent with `AP_HAL::Device::register_periodic_callback`.
   Measure actual servicing latency; do not derive timing from the comment.
4. AP_Beacon defaults to four beacons. EKF3's `BeaconFusion::lastTime_ms` is
   explicitly sized `[4]`; increasing the frontend constant alone will fail its
   existing size assertion. BCN logging also has only D0 through D3.
5. AP_DAL_Beacon records count in its constructor, whereas beacon discovery can
   occur later. Count initialization and refresh must be tested and corrected
   as needed alongside eight-anchor support and replay.
6. AP_Beacon distance publication stamps delivery time. Forwarding cached data
   repeatedly would falsely keep it fresh. Publication must preserve acquisition
   time and occur once per accepted observation.
7. EKF3's beacon path has a range-based alignment filter, but its automatic origin
   and beacon-height readiness also consult the backend's reported vehicle
   position. An adapter publishing only distances is not a complete startup plan.
8. Copter initializes AP_Beacon before AP_Ranging and schedules beacon updates at
   400 Hz versus AP_Ranging at 50 Hz. The adapter must tolerate initialization
   order and use a coherent observation snapshot.
9. cus-node-firmware already has a four-message TWR anchor, an algorithm table,
   persistent TLV configuration, USB console, and Unity/CMock tests. Its current
   TWR path applies a software antenna offset while `uwb.c` sets zero hardware
   antenna delay. That convention differs from AP_Ranging and must be reconciled.

## Anchor parameters and coordinate system

Use eight fixed parameter subgroups, with slot number equal to anchor ID. Each
subgroup contains `AP_Int8 enable` and `AP_Vector3f position`. Vector components
are exposed as ordinary scalar parameters by AP_Param.

| Proposed parameter | Default | Meaning |
| --- | --- | --- |
| `RNG_MODE` | 0 | 0 peer TWR, 1 anchor TWR; `RNG_TYPE=0` still disables the device |
| `RNG_A1_EN` through `RNG_A8_EN` | 0 | Explicit enable for the corresponding fixed anchor ID |
| `RNG_A1_POS_X/Y/Z` through `RNG_A8_POS_X/Y/Z` | 0 | Surveyed antenna position in meters, X north, Y east, Z down |
| `RNG_NET_ID` | 1 | Network identifier shared by the new peer and anchor protocol |
| `RNG_A_POLL_MS` | provisional 10 | Minimum interval between starting successive anchor exchanges, not per-anchor period |
| `RNG_A_TMO_MS` | provisional 30 | Entire anchor exchange deadline, including REPORT |

Use existing `RNG_REPLY_US`, `RNG_CHAN`, and `RNG_ANT_DLY` for their existing
physical meanings. Keep `RNG_PEER_1..4` as the peer roster and preserve parameter
indices. Peer mode uses `RNG_POLL_MS` for initiation cadence and `RNG_XCHG_MS`
for the whole exchange, now including REPORT. Revalidate timing defaults and
reject combinations that cannot accommodate four messages; unchanged parameter
values do not imply unchanged on-air timing. Anchor mode ignores the peer roster
and peer polling interval. Apply mode, identity, anchor map, RF, and transaction
timing configuration at boot for v1;
all new parameters are reboot-required. Document the boot snapshot and updated
reboot requirements for reused timing parameters in both modes. Detect pending
relevant parameter changes and require reboot before arming with anchor navigation;
never partially apply a new map.

Proposed allocation: `RNG_MODE` at group index 14, `RNG_NET_ID` at 15, eight anchor
subgroups at 16 through 23, and anchor timing at 24 and 25. Each anchor subgroup
uses child indices 0 for enable and 1 for position. Verify AP_Param nesting,
metadata, generated vector component names, and storage compatibility before
freezing this allocation. Do not reuse retired indices. All resulting parameter
names must be at most 16 characters.

An empty slot means `EN=0`, regardless of coordinates. `(0,0,0)` is a valid enabled
anchor location. Set all three coordinates before enabling a slot. Parameter
write history cannot reliably distinguish an intentional zero from an unentered
zero; validate the resulting map rather than relying on write history.

For example, enabling anchors 1, 3, 6, and 8 polls exactly those IDs. A missing
configured anchor remains eligible for periodic retry but contributes no fresh
range. A disabled anchor is never polled or accepted as a navigation observation.

Survey positions describe the UWB antenna phase centers in one common local NED
frame. Use `BCN_ORIENT_YAW=0` for the initial NED setup and configure the existing
`BCN_LATITUDE`, `BCN_LONGITUDE`, and `BCN_ALT` origin consistently on every vehicle.
Define positive-down examples explicitly: an anchor two meters above the local
origin has Z = -2. Do not silently reuse Bitcraze Z-up coordinates.

Keep the surveyed map authoritative on the vehicle. Nodes need their ID, network,
RF profile, and calibration; they do not need to know their position to return
TWR timestamps. Existing node position storage can remain for legacy modes, but
v1 ArduSwarm reports do not override the vehicle's survey.

## Geometry and navigation validity

Four non-coplanar anchors are the normal minimum for an unambiguous unconstrained
3D range solution. Four entries alone are insufficient: coplanar arrangements
can have mirror ambiguity and poor vertical conditioning. More anchors improve
redundancy but do not fix a uniformly poor layout.

Separate these states in code and diagnostics:

- Configured: enabled map slots with finite, bounded coordinates.
- Reachable: an anchor has recently completed an exchange.
- Range valid: the observation passes timing, calibration, and numeric checks.
- Navigation ready: sufficient fresh independent ranges, usable geometry,
  successful bootstrap/alignment, and a valid origin/heading configuration.

Allow one to three enabled anchors for bench ranging and calibration. When the
beacon navigation source is selected, require at least four enabled distinct
positions and usable 3D geometry for this release. Do not publish a valid 3D
bootstrap fix with fewer than four fresh usable anchors. Deliberate 2D operation
with a separate height constraint is a later configuration, not an implicit
fallback that fabricates Z.

Check configured geometry with a rank/conditioning test and reject duplicates,
nonfinite coordinates, and degenerate layouts. Check current observation geometry
again in the bootstrap solver. Establish conditioning, residual, and uncertainty
thresholds using deterministic tests and recorded data, then document them.
If fewer than four good anchors remain during operation, mark 3D readiness lost.
Specify and test whether the adapter gates all aiding or supplies remaining raw
ranges while EKF uncertainty grows; for v1 use conservative all-aiding gating
until sufficient geometry returns. Explicitly invalidate adapter health/ranges
on readiness loss; do not let an older healthy snapshot bypass this gate. Preserve
diagnostics during the outage and let existing EKF timeout/failsafe logic operate
without refreshing stale data.

## Four-message protocol

Use one ArduSwarm-specific wire format for drone-to-drone and drone-to-anchor
ranging. The exchange roles (initiator/responder) are independent of identity
roles (tag/anchor). A drone can take either exchange role in peer mode; a drone
initiates and a fixed anchor responds in anchor mode. This replaces the old peer
radio protocol; it is also not wire-compatible with Bitcraze's MAC packets.

```text
Initiator A (drone)                       Responder B (drone or anchor)
POLL ----------------------------------> capture t2 = poll_rx
capture t4 = response_rx <--------------- RESPONSE, capture t3 = response_tx
FINAL [t1, t4, scheduled t5] ------------> capture t6 = final_rx; compute range
capture REPORT <------------------------ REPORT [status, t2, t3, t6]
compute range from local t1/t4/actual t5 and reported t2/t3/t6
```

A captures actual t1=POLL TX and t4=RESPONSE RX. FINAL includes these and its
hardware-scheduled t5=FINAL TX time. B now has all six times and computes the range.
REPORT returns B's actual timestamps so A computes the range independently using
its actual FINAL TX timestamp. Use the same DS-TWR math and acceptance checks at
both ends, not a transmitted floating-point distance as the sole result.

FINAL must be sent with DW1000 delayed TX. Derive t5 from the exact quantized,
antenna-delay-adjusted scheduled timestamp returned by the driver, never from a
software-clock estimate. The inspected libdw1000 `dwSetDelay` clears the lower
nine scheduling bits and adds configured antenna delay; confirm this behavior
against the pinned dependency and hardware. Compare actual FINAL TX with the
embedded t5 before A accepts a range. Missed scheduling must abort, with no
immediate-TX fallback carrying stale t5. Handle 40-bit wrap and callback ordering.
A discrepancy is a protocol/timing fault: A rejects and logs it; B cannot retract
an already published result because no acknowledgement exists. Validate this
scheduled-versus-actual equality as a release gate for two-sided publication.

B publishes at most once after a valid FINAL and complete local timestamp capture.
A publishes at most once after a valid REPORT and verified FINAL TX. REPORT TX
completion is not proof of reception at A. If REPORT is lost, B may have a valid
range while A times out; keep independent freshness and completion diagnostics.
Both results describe the same physical exchange and are correlated. A companion
must not treat them as independent measurements merely because both agents report
them. Other listening agents do not acquire this pair's range as a local range.

Proposed wire header, encoded explicitly in little endian:

| Offset | Bytes | Field |
| --- | --- | --- |
| 0 | 2 | Magic `0xA752`, proposed shared ArduSwarm TWR marker |
| 2 | 1 | Version 1 |
| 3 | 1 | Type: 1 POLL, 2 RESPONSE, 3 FINAL, 4 REPORT |
| 4 | 2 | Network ID |
| 6 | 1 | Source role: 1 tag, 2 anchor |
| 7 | 1 | Source ID |
| 8 | 1 | Destination role |
| 9 | 1 | Destination ID |
| 10 | 2 | Transaction sequence |
| 12 | 4 | Initiator boot/session nonce, echoed throughout the exchange |

POLL and RESPONSE contain the 16-byte header only. FINAL appends three 5-byte
DW1000 timestamps t1, t4, scheduled t5, totaling 31 application bytes. REPORT
appends one status byte and three 5-byte timestamps t2, t3, t6, totaling 32
application bytes. All lengths exclude PHY FCS. Status zero means valid; reject
unknown or nonzero status in v1. Timestamp zero is not a missing-value sentinel because
it can occur at counter wrap. Invalid transactions normally time out without a
report. Freeze constants only after cross-repository fixtures agree.

Tag IDs are `MAV_SYSID` values 1 through 255. Anchor IDs are 1 through 8. Roles make
(tag, 1) distinct from (anchor, 1); do not apply peer-mode self-ID exclusion to
anchors. No broadcast ranging in v1. Network ID separates installations; it is
not authentication. The session nonce prevents accidental acceptance across tag
reboots; it is not a security token. Sequence wrap is safe only with an active
bounded exchange lifetime and full transaction matching.

Validate exact packet length, magic, version, type, network, role/ID pair, enabled
anchor or admitted peer, initiator session, sequence, expected state, and deadline
before touching timestamp state. Only accept tag-to-tag transactions in peer mode
and tag-to-anchor transactions in anchor mode. Validate directions per message. Encode/decode bytes without packed-struct casts. Check the driver's FCS
length convention and oversize rejection on both platforms. Do not reinterpret
legacy Bitcraze or peer frames as new packets. Retain radio CRC filtering.

Use raw application framing consistent with the current AP_Ranging transport;
the new node algorithm bypasses legacy `packet_t` MAC parsing. Configure matching
channel, bitrate, PRF, preamble length/code, SFD, CRC, and frame filtering. Start
with the existing short-data fast-accuracy profile, channel 2 and preamble code 9;
make channel configurable on the node. Preserve RF settings for legacy modes.

## Range calculation and calibration

Define the six event times as initiator t1=POLL TX, t4=RESPONSE RX, t5=FINAL TX;
responder t2=POLL RX, t3=RESPONSE TX, t6=FINAL RX. Form modular 40-bit differences:

```text
Ra = t4 - t1     Da = t5 - t4       (initiator clock)
Rb = t6 - t3     Db = t3 - t2       (responder clock)
ToF_ticks = (Ra*Rb - Da*Db) / (Ra + Rb + Da + Db)
range_m = ToF_ticks * DISTANCE_OF_RADIO
```

Mask each difference with `0xFFFFFFFFFF`; never subtract absolute timestamps from
different devices. Retain the existing alternative DS-TWR formulation, which
allows unequal reply delays. Validate each interval against the exchange bound
BEFORE multiplication. Use sufficiently wide arithmetic and explicitly test
near-equal products, sign, overflow bounds, zero denominator, nonfinite results,
and negative/out-of-range distances. The current peer algorithm's -1 m lower
limit must not silently become the navigation acceptance rule. Define a calibrated
positive operating range for the shared new protocol; log rejected results
separately. Publish neither peer nor anchor navigation ranges from invalid math.

Use the same documented hardware antenna-delay convention on tag and anchor in
the new mode. Disable the Bitcraze software `ANTENNA_DELAY/2` adjustments in that
mode so delay is not corrected twice. Provide persistent node antenna-delay
configuration and retain legacy calibration behavior for legacy algorithms.
Calibrate each radio at known distances and verify the interpretation against
actual libdw1000 TX/RX timestamp registers. A shared default delay is an initial
value, not evidence of calibrated accuracy.

Assign each endpoint an acquisition timestamp near the exchange midpoint using
its own HAL clock and captured exchange timing, not REPORT arrival or frontend
delivery. Endpoint HAL clocks are not synchronized: correlate a pair of reports
with initiator ID, session, and sequence, not by comparing their local times.
Bound the approximation and account for it in measurement delay/noise. DW1000
40-bit ticks remain local to range math; do not expose them as AP_HAL timestamps.

## State machines and channel sharing

Common initiator states: idle/backoff, wait RESPONSE, wait FINAL TX completion,
wait REPORT. Track POLL TX completion as an event prerequisite as well. Common
responder states: idle, wait RESPONSE TX completion, wait FINAL, wait REPORT TX
completion. Peer drones implement both; anchor nodes implement the responder. Store only one
active transaction per radio, keyed by both role/ID pairs, network, session, and
sequence. Handle already-latched TX and RX events without overwriting timestamps.

Every responder ignores competing POLLs while busy; it must not replace the
active initiator or its timestamps. A peer drone initiating an exchange also
ignores competing POLLs until it returns idle. Ignore duplicates or out-of-order frames without extending the
transaction deadline. Timeout, missed delayed TX, RX error, failed TX, and radio
reset clear state and restore reception. A lost REPORT causes a new transaction
with a new sequence at A, never re-publication of the old measurement. It does
not invalidate an already accepted responder-side sample. Keep REPORT delivery
bounded even though its own TX timestamp is unnecessary.

In peer mode, use the configured roster to admit both outbound and inbound
transactions. Configure peer links mutually; reject self IDs and unlisted peers
before they consume table slots. Preserve stable peer slots. Each accepted range,
whether initiated locally or remotely, updates the local peer's freshness once.
A received responder-side sample can defer the next redundant outbound poll to
that peer; it must not indefinitely suppress other peers. Both ends remain able
to initiate after timeout or stale data, without a permanent lower-ID owner.
Keep randomized initiation/backoff to break simultaneous-POLL collisions and
validate fairness and freshness after this optimization. A second reciprocal
exchange is not required merely to give both endpoints a distance.

Select anchors round-robin from the enabled map with randomized initial phase and
bounded per-transaction jitter. Back off unavailable anchors with a bounded probe
interval while continuing to service reachable ones. No dynamic allocation or
blocking delays in the radio callback. Keep existing IRQ-gated SPI servicing and
bus semaphore rules. Size the callback budget against shared SPI devices.

Provisional bench timing is 3 ms replies, 10 ms minimum start spacing, 0 to 5 ms
jitter, and 30 ms total exchange timeout. Three reply gaps plus airtime/service
latency make a successful exchange roughly 10 to 15 ms; a loss-free eight-anchor
sweep may therefore be roughly 80 to 120 ms before contention. These are budgets
to measure, not guaranteed rates. Validate timeouts against configured reply delay
and actual CPU/bus latency. Do not reuse the peer default 50 ms plus 0 to 49 ms
jitter: eight starts could span roughly 400 to 800 ms, exceeding the current
300 ms beacon freshness limit.

Record per-anchor age, complete sweep time, success ratio, and tag fairness for
1, 2, and 4 tags. Require useful fresh geometry inside the 300 ms age window for
the declared supported configuration. If this is not sustained, reduce the
supported tag count or implement an explicit scheduler in a later increment;
do not hide capacity limitations by making stale samples look current.

## AP_Ranging implementation

1. Add `Mode` and an eight-slot anchor configuration table. Keep
   `AP_RANGING_MAX_NODES=4` and the current peer table separate from the new
   `AP_RANGING_MAX_ANCHORS=8` table.
2. Extract a common four-message codec, range-math helpers, and transaction
   engine from the DW1000 transport. Keep one SPI device and one radio callback.
   Separate mode policies (roster, scheduling, publication) from shared initiator
   and responder state handling; avoid parallel copies of the ranging algorithm.
3. Define an immutable observation with remote role/ID, distance, acquisition
   time, receipt time, local exchange role, initiator ID/session/sequence, and
   validity/quality. Use a bounded synchronized
   queue from the bus thread to main-thread publication. Drop/count overflow with
   a documented policy; never block the radio thread.
4. Maintain separate anchor snapshots for the beacon adapter and diagnostics.
   Main-loop service frequency must drain the measured arrival rate with margin;
   an adapter must not drain the queue independently of the frontend.
5. Preserve the public peer `NodeState` interface, peer log layout, and version-1
   peer TUNNEL layout while updating their values from either exchange role. Do not
   label an anchor as a vehicle peer or publish anchor-mode values in that format.
6. Add per-observation logs for both modes and state/timeout/queue counters.
   Include the transaction key and local exchange role to correlate both ends;
   keep legacy peer snapshots for existing consumers. The version-1 peer TUNNEL
   snapshot has no transaction key, so use the new diagnostic record when
   deduplication across agents is required. Design a separately versioned anchor
   telemetry record if companion forwarding is in
   scope; make mode, ID, time/age, validity, and units explicit. Version-1 companion
   readers must continue to receive valid peer records only in peer mode.
7. Separate transport health from navigation readiness and expose clear reasons
   such as invalid survey, insufficient fresh anchors, and exchange timeout.

## AP_Beacon and EKF3 integration

Add an `AP_Beacon_Ranging` backend and a new `BCN_TYPE` value (proposed 4, currently
unused in this base). This backend consumes AP_Ranging and owns no transport.
Do not let its base constructor open a Beacon UART unnecessarily: add a minimal
opt-out with the existing UART behavior retained for other backends.

At startup, derive a stable dense beacon-index mapping from ascending enabled
anchor IDs. Example: IDs 1, 3, 6, 8 map to instances 0, 1, 2, 3. Keep physical IDs
in anchor diagnostics. Disabled slots must not create phantom beacons at origin,
contribute to counts, or enter the beacon fence. Never remap a timed-out anchor
within a running session. Reboot applies map changes.

Initialize the complete configured map before EKF consumes it. Account for current
Copter init order by initializing static map metadata without needing an active
radio; poll for driver readiness during update. Review AP_DAL construction order,
refresh its count from AP_Beacon, and ensure EKF fusion-report allocation is sized
safely when the map becomes available. Test a map populated after DAL construction.

Support eight beacons in frontend storage, DAL/replay, EKF last-update arrays,
report allocation, loops, and diagnostics. Size EKF3's fixed `[4]` array from the
beacon capacity. Audit EKF2 and other existing beacon consumers for the same
assumption. Enable the eight-slot capacity in builds with the new adapter, leaving
unrelated small builds at their existing default where practical. Add bounds
checks on replay instance indices. Preserve old log formats; new per-anchor logs
cover IDs 5 through 8 rather than silently truncating observability.

Add a backward-compatible timestamped distance publication path so this backend
preserves acquisition time. Existing vendor backends can retain the current
setter behavior. Publish each sample once. Distinguish received validity from
nonzero timestamp so boot-time samples and missing data cannot masquerade as one
another. At first use, vehicle-position validity must require an actual fix, not
an initially zero timestamp falling within the timeout window.

EKF3 currently uses global beacon measurement noise/delay. Start with calibrated
`EK3_BCN_M_NSE` (meters), `EK3_BCN_I_GTE` (percent of standard deviations),
and `EK3_BCN_DELAY` (milliseconds), verified against the implementation baseline.
With acquisition timestamps preserved, set delay for remaining unmodeled latency
only; do not subtract transport latency twice. Do not claim per-sample variance
is consumed unless that interface is actually extended. Keep the default
global-noise path for v1 and log quality for rejection/analysis.

### Position bootstrap and origin

Add a bounded UWB-only multilateration helper owned by the beacon adapter for
startup position and readiness. It uses at least four fresh ranges and the survey,
with rank checks, bounded iterations, residual rejection, and a conservative
position error estimate. Initialize from the linear system formed by subtracting
squared range equations, then refine a weighted nonlinear least-squares objective.
Use a conditioned solve rather than an unchecked matrix inverse. Tests must cover
both the linear seed and refinement failure; never publish the last iterate as a
valid fix merely because the iteration limit was reached. It must not obtain its
answer from the main EKF or SITL truth. Refresh only on a genuinely new usable
observation set; account for sample age spread and motion, and reject sets that
cannot support the claimed accuracy.
Use the result via `set_vehicle_position` for the existing alignment/origin path,
not as a second ExternalNav measurement stream. The main EKF continues to fuse raw
ranges, avoiding a second fusion of the same data as an independent position.

Review and test the existing range-only alignment filter and its relationship to
this reported position before hardware navigation. Verify cold startup with no
GPS and no inherited EKF origin, including all validity flags and origin altitude.
If the existing beacon-height path requires a focused correction, isolate it with
its own regression test and human estimator review rather than masking readiness.

Initial integration tests use `EK3_SRC1_POSXY=4`, velocity sources 0 unless another
real velocity sensor is configured, and an independently valid heading source.
Test barometer/rangefinder height separately from full `EK3_SRC1_POSZ=4` beacon
height. Full 3D beacon-height support is an acceptance target, not something
inferred from successful XY flight. One tag does not measure yaw.

For v1 constrain the tag antenna offset from the EKF position reference to a
measured negligible value within the declared error budget. The current beacon
range model does not explicitly correct a body-frame antenna lever arm. Supporting
larger offsets requires a separately tested attitude-dependent observation model;
do not subtract the norm of the offset from every range.

Beacon setup can also affect fence/avoidance boundaries. Audit this side effect
and test disabled/sparse anchors; do not turn on avoidance merely by enabling
ranging.

## cus-node-firmware implementation

Create an independent ArduSwarm anchor algorithm, for example
`src/uwb_arduswarm_anchor.c` with an explicit protocol header. Register it by
appending a new mode to `inc/uwb.h` and `src/uwb.c`; preserve legacy mode numbers.
Update the USB console in `src/main.c` to select/display the new mode and settings.

Implement the shared responder transaction lock, four-message exchange, FINAL
timestamp decoding, local range calculation, REPORT generation, packet validation,
recovery, and matching RF configuration described above. Expose anchor-side range
diagnostics keyed by drone MAV_SYSID; a fixed anchor does not need to initiate
exchanges to observe distances to tags. Never emit raw legacy MAC packets from the ArduSwarm algorithm. Keep board initialization,
FreeRTOS event dispatch, and libdw1000 HAL rather than rewriting them wholesale.

Add persistent settings for anchor ID 1 to 8 (reuse address storage with mode-local
validation if appropriate), network ID, channel, antenna delay, reply delay, and
exchange timeout. Provide console readback and diagnostics. Reject invalid IDs in
ArduSwarm mode instead of silently adopting legacy address 0. Keep legacy defaults
usable in legacy modes; entering the new mode requires an explicit valid setup.

Append configuration field IDs without renumbering existing TLV fields. Audit the
100-byte configuration buffer, serialized sizes, bounds checks, checksum, missing
fields, and power-interrupted writes before adding fields. Define migration/default
behavior for older images and test save/reboot/restore. Increase storage only with
verified EEPROM layout and memory budget. Update configuration tools, help, and
protocol docs together. Disable or reject legacy LPP changes that could silently
change active ArduSwarm identity/mode mid-transaction; v1 management stays on USB.

Keep protocol test vectors identical in both repositories. Use a portable C codec
or independently implemented codecs against shared golden fixtures; do not add a
new submodule dependency just to share a small header. Record protocol version and
compatible firmware revisions in deployment documentation.

## Files and responsibility boundaries

| Area | Planned files or locations |
| --- | --- |
| Parameters and observations | `libraries/AP_Ranging/AP_Ranging.{h,cpp}`, backend and config files |
| Radio and shared transaction engine | `AP_Ranging_DW1000.{h,cpp}`, common protocol/math/state helpers, peer and anchor policies |
| Unit tests and fixtures | `libraries/AP_Ranging/tests/` and matching node `test/` fixtures |
| Beacon adapter and bootstrap | New `libraries/AP_Beacon/AP_Beacon_Ranging.{h,cpp}`, bounded solver helper/tests |
| Capacity and publication | AP_Beacon frontend/backend/config; AP_DAL_Beacon and replay |
| Estimator integration | EKF3 beacon array sizing and necessary startup fixes; audit EKF2 |
| Vehicle/build integration | Copter initialization/scheduler/arming checks, build options, board feature selection |
| Logging | AP_Ranging LogStructure and logger registration; preserve existing formats |
| Simulation | AP_Ranging SITL backend, targeted Copter autotests and replay checks |
| Node algorithm/configuration | `inc/uwb.h`, `src/uwb.c`, new algorithm/codec, `cfg.*`, `main.c`, Makefile |
| Node tests and usage | Unity/CMock tests, protocol docs, configuration and calibration guide |

Use compile-time guards for anchor ranging and the ranging beacon adapter, with
custom-build options and explicit dependencies. Peer-only builds must not require
AP_Beacon. Beacon builds without AP_Ranging must continue compiling. On Crazyflie,
explicitly enable only the necessary beacon backend and compare flash, RAM, stack,
and CPU budgets against the same master baseline. Do not change submodule contents.

## Peer protocol migration

Replace the existing 0xC1/0xC2/0xC3 peer frames with the shared versioned
four-message format. All participating peer vehicles must be upgraded together
and use the same network/RF profile. No silent legacy fallback or mixed-protocol
negotiation in v1. Old radio frames are rejected and cannot refresh health. Provide
an explicit deployment note and a rollback procedure using a consistent firmware
set for the fleet. Do not claim interoperability with unupgraded vehicles.

Retain peer mode as `RNG_MODE=0`, peer identities from MAV_SYSID, existing parameter
indices, and companion snapshot/log layouts where their meaning is unchanged.
Document changed semantics: either exchange role now produces a local sample,
mutual peer admission is required, REPORT extends the transaction deadline, and
new timing settings apply on reboot. Avoid interpreting a faster stream of
correlated opposite-end reports as additional independent observations.

## Implementation sequence and acceptance gates

1. **Freeze common parameters and protocol.** Add configuration validation,
   codec, range math, and cross-repository fixtures. Gate: storage indices and
   names validated; role overlap works; FINAL=31 and REPORT=32 bytes; existing
   companion snapshot layouts preserved; radio incompatibility documented.
2. **Upgrade peer TWR first.** Implement shared initiator/responder logic on two
   drones. Gate: a single successful four-message exchange updates both endpoints,
   actual FINAL TX matches its scheduled timestamp, no reverse exchange required,
   and lost REPORT updates B only without refreshing A or fabricating completion. Test
   simultaneous polls, roster admission, duplicates, reboot, and three/four peers.
3. **Add node responder and anchor policy.** Reuse the shared protocol with one
   node and one tag, navigation disabled. Gate: identical golden fixtures across
   repositories, node and tag ranges agree within numeric tolerance, calibrated
   physical distances agree within the declared measurement budget.
4. **Scale to eight configured anchor slots.** Add roster scheduling, queue,
   snapshots, diagnostics, and sparse-map handling. Gate: only enabled anchor IDs
   are polled; missing anchors recover; every anchor has age/validity diagnostics;
   peer mode still works after the common-engine changes.
5. **Integrate beacon capacity and bootstrap.** Add adapter, timestamp preservation,
   count/replay fixes, multilateration, and readiness checks. Gate: cold GPS-free
   SITL startup and full 3D geometry tests, without truth-position injection.
6. **Validate vehicle navigation.** Build both Crazyflie variants, replay logs,
   perform static and moving bench comparisons, then controlled low-speed flight
   after review of estimator behavior. Gate: declared accuracy, latency, dropout,
   and recovery limits achieved with raw evidence retained.
7. **Validate shared-anchor operation and fleet migration.** Test two/four tags,
   boot orders, overlapping tag/anchor IDs, absent anchors, mixed RF load, and
   rejection of legacy peer packets. Gate: per-tag freshness/fairness support the
   declared configuration and firmware migration instructions are reproducible.

Each increment should be independently reviewable. Commit descriptions must
reflect actual changes with subsystem prefixes and AI-assistance disclosure in
contribution documentation. Before any upstream PR, check for overlapping work,
follow the repository template, and obtain required maintainer discussion/review
for estimator changes. No PR, firmware upload, or flight is part of this plan task.

## Test matrix

| Layer | Required cases |
| --- | --- |
| Parameters | All slots disabled; one anchor at origin; IDs 1/3/6/8; eight enabled; invalid flags; duplicate coordinates; NaN/Inf; reboot persistence; pending map edit |
| Geometry/solver | Exact 3D synthetic positions; noisy overdetermined data; four coplanar anchors; near-degenerate geometry; one outlier; stale/motion-skewed sets; no valid initial guess |
| Codec | Golden bytes both directions; every short/oversize length; bad magic/version/type; swapped roles; network mismatch; tag 1 with anchor 1; tag 255; unknown anchor |
| Timestamp math | Unequal delays; independent clock offsets/drift; 40-bit wrap; sequence wrap; zero timestamp; bounded large intervals; product overflow rejection; calibration sign |
| State machines | Both ends update from one exchange; lost REPORT updates B only and does not refresh A; no reverse poll required; duplicates; reordering; old session; busy drone/anchor; simultaneous polls; missed delayed TX; scheduled/actual FINAL mismatch; RX error; reset; callback ordering |
| Data handoff | Concurrent producer/consumer; both exchange roles; queue overflow; transaction deduplication; independent endpoint clocks; no repeated freshness refresh; acquisition vs delivery delay; missing first sample; millisecond wrap |
| Beacon/DAL | Eight-slot capacity; sparse dense mapping; late map initialization; correct physical-ID diagnostics; replay bounds; raw timestamp preservation; existing backend regression |
| SITL/EKF | Cold GPS-free startup; independently computed bootstrap; XY plus separate height; beacon 3D height; yaw setup; origin consistency; loss/recovery of geometry; no stale aiding |
| Compatibility | Four-message peer default; existing parameter indices and peer log/TUNNEL layouts; reject old peer frames; fleet upgrade/rollback; mutual roster admission; legacy node modes; optional features disabled; no UART/fence side effects |
| Hardware | Known ranges and surveyed points; varied heights/orientations; shared SPI load; occlusion/NLOS; anchor reboot; tag reboot; 1/2/4 tags and fair access |

ArduPilot: run relevant new GTests, Copter SITL tests, existing
`test.Copter.BeaconPosition`, beacon replay coverage, and builds for `sitl`,
`crazyflie2`, and `crazyflie2_bl` with appropriate feature profiles. Use `./waf`
from the repository root. Node: build with the existing Makefile and run
`./tools/do test` plus new Unity/CMock cases. Check formatting, parameter metadata,
Python lint where applicable, compile-time feature combinations, and binary sizes.
Initialize required submodules in the isolated checkout before builds without
modifying their contents or importing unrelated dirty submodule work.

Before flight, freeze measured acceptance limits for range error, position error,
vertical error, measurement age, initialization time, and recovery time using the
intended room, anchor survey, and tag count. Provisional timing budgets above are
not test results. Validate beacon-height and yaw behavior independently; successful
bench ranging alone does not establish navigation readiness.

## Reference material

- Local AP_Ranging, AP_Beacon, AP_DAL_Beacon, and EKF3 sources at the branch base.
- Local cus-node-firmware sources at the revision listed above, especially
  `src/uwb_twr_anchor.c`, `src/uwb.c`, `src/cfg.c`, and `inc/cfg.h`.
- [Bitcraze four-message TWR protocol](https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/protocols/twr-protocol/)
  for the message sequence and anchor timestamp report concept.
