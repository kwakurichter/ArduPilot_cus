Two-Crazyflie GUIDED backward-and-return test
==========================================

One host process controls MAV_SYSID 1 and 2 over ONE multi-vehicle Crazyradio
bridge. Defaults: take off to nominal 1 m, hover together for 3 seconds, move
backward 1 m, return to each aircraft's own captured hover position, then LAND.
Both aircraft must complete each stage before either starts the next stage.
Backward is relative to each aircraft's heading captured just before arming.
They need matching physical headings if you want parallel flight paths.

This is a host-side script. No new firmware is needed. It does not synchronize
radio delivery to sub-millisecond precision or provide formation control,
collision avoidance, or a shared spatial reference between aircraft.

Known height-reference issue remains
------------------------------------
The prior single-vehicle flight commanded 1 m but hovered around 2 m and peaked
at 2.63 m. This script retains the existing home/local-NED altitude behavior;
--height 1 is NOT a guarantee of 1 m clearance from the floor. Its height checks
use the EKF estimate and cannot detect that same reference error. This issue is
not fixed by adding a second vehicle. Allow for those observed heights and keep
the two backward/return flight corridors separated.

A subsequent user-performed paired flight completed the full sequence with the
updated bridge and Crazyradio USB firmware. The nominal 1 m request still gave
mean downward ranges of about 1.63 m and 1.27 m during the movements, with
takeoff peaks of 2.15 m and 1.71 m. See guided_pair_validation.txt for evidence.

Connection
----------
Use the updated multi-vehicle mavlink_bridge.py with both aircraft radio URIs,
and point its one UDP destination at 127.0.0.1:14560. Preserve your actual radio
indices, channels, data rates and addresses. Each aircraft must have a unique
MAV_SYSID and radio address. For example, with your two real URIs substituted:

  python3 /Users/kwaku/Documents/Crazyflie/nrf51/nrf-firmware-cus/tools/mavlink_bridge.py \
    --uri YOUR_FIRST_RADIO_URI --uri YOUR_SECOND_RADIO_URI --udp 127.0.0.1:14560

QGC listens on 127.0.0.1:14550. This script relays telemetry and QGC commands:

  one multi-vehicle bridge <-> guided_pair.py :14560 <-> QGC :14550

Stop any earlier guided_flight.py/guided_pair.py instance before starting this
one: both use the same listening port. Do not connect a second sequencer to the
bridge. Separate bridge processes feeding this port are not supported.

Run from the directory containing guided_pair.py and guided_flight.py, using a
Python environment with pymavlink installed (the existing sequencer environment):

  python3 guided_pair.py --target-systems 1 2

This previews the plan without opening sockets or transmitting commands.
To execute, use a new log path for each run:

  python3 guided_pair.py --target-systems 1 2 --height 1 --hover 3 --distance 1 \
    --execute --log /tmp/guided-pair-01.jsonl

The log is created exclusively; an existing file is never overwritten.
The shared GCS source system defaults to 255 and component to 191. It must be
permitted by BOTH aircraft's GCS filtering configuration. --source-system can
change it but cannot equal either aircraft's system ID.

Behavior
--------
Before arming, both heartbeats must be fresh. Each aircraft is checked disarmed,
configured for telemetry, given/reuses its EKF origin and home, switched to GUIDED,
and checked for prearm readiness. No PID, thrust, arming-check or failsafe
parameters are changed. Message stream rates are requested and not restored.
Origins default to the existing single-vehicle script's synthetic indoor
coordinates when none already exists; these are not real geographic coordinates.
--lat/--lon/--amsl may specify a real origin, with existing-origin consistency checks.

Normal arm commands are sent to both; both must confirm arming before takeoff
commands are sent. The script does not force-arm or retry an ambiguous arm.
Both must reach their local takeoff targets and have usable horizontal EKF
estimates before movement. The first hover uses each aircraft's then-current
horizontal position and original takeoff height; all later targets use that
fixed position. Position/yaw setpoints are sent to both at 5 Hz throughout each
hover/movement stage. Default arrival limits are 0.30 m 3D error and 0.40 m/s
speed, with both continuously settled for 3 seconds at the first hover and
1 second at the backward and return points. These are inherited coarse test
limits, not a precision-positioning claim.

Options include --hover, --height, --distance, --hold, --position-tolerance,
--settle-speed, --takeoff-timeout, --move-timeout, --listen, --qgc and --log.
--speed is the inherited pre-takeoff speed request: position-mode entry resets
it in this firmware, so it is NOT an effective movement speed limit.

On rejection, timeout, lost usable telemetry, or first Ctrl-C, stop the group
sequence. For each script-controlled aircraft still reporting fresh GUIDED
state with no pilot takeover, send one best-effort LAND. A lost link cannot
confirm delivery; stale aircraft receive no blind commands. An aircraft already
in LAND remains there. A QGC mode/flight command for one vehicle preserves that
pilot takeover and requests LAND only from the other eligible aircraft; a
broadcast takeover yields both. No force-disarm is sent.

LAND is confirmed by ACK or a new heartbeat reporting LAND, so a lost ACK does
not alone stop landing monitoring. Completion requires both aircraft confirmed
on ground and disarmed. Normal disarm is sent only after 2 seconds continuously
on ground; autopilot auto-disarm is accepted. Unconfirmed disarm is not retried.

After completion or abort, the QGC relay remains running, without script GCS
heartbeats. Ctrl-C again closes it. Keep QGC connected to maintain its own link.

Validation
----------
  python3 -m unittest discover -s companion -p 'test_guided*.py'

Tests include shared UDP routing, isolated telemetry/ACKs, heading-relative
backward targets, stage synchronization, rejected arming, stale telemetry,
QGC takeover, lost LAND ACK and confirmed-ground-only disarming. The deterministic
kinematic tests do not model flight dynamics. See guided_pair_validation.txt for
actual SITL results. No physical multi-vehicle flight has been performed by the
assistant. AI-assisted implementation and tests.
