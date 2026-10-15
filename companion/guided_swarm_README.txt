GUIDED demos for 1–4 Crazyflies
==============================

guided_swarm.py is the single, self-contained entry point. It replaces
guided_flight.py, guided_pair.py and the old guided_trajectory.py prototype.
Requires Python 3.9+ and pymavlink. Without --execute it prints the plan, opens
no sockets and sends no commands. No firmware or aircraft parameters are
changed by the script. Implementation and tests are AI-assisted.

Connection and launch positions
-------------------------------
Use one updated mavlink_bridge.py process, including both dongles if needed,
with UDP destination 127.0.0.1:14560. QGC listens on 127.0.0.1:14550. Stop any
previous guided script using port 14560 before starting this one. Do not feed
separate bridge processes into the same script: it pins one bridge endpoint.

  one bridge / one or two dongles <-> guided_swarm.py :14560 <-> QGC :14550

--target-systems accepts one to four distinct vehicle IDs; default is 1.
Their order assigns the printed launch marks. Place aircraft on a level floor
at those marks, all facing the SAME physical forward direction. Each aircraft
has its own EKF origin and captured yaw; estimated displacements are mapped
onto the marked layout. The script does not move aircraft into formation.

  1 aircraft: centre.
  2 aircraft: side by side, default 1.50 m apart.
  3 aircraft: equilateral triangle, default 1.50 m sides, first ID in front.
  4 aircraft: square, default 1.25 m sides and 1.77 m diagonals:

          FORWARD
     first       second
     third       fourth      RIGHT ->

--spacing changes the side length. Nominal pairwise distances must be within
1.15–1.85 m. The active estimated-separation limit is 1–2 m. This is based on
drifting estimates, not measured physical separation or collision avoidance.

Patterns and commands
---------------------
Run from companion/ using your pymavlink environment. Use a NEW log filename
for every flight; existing logs are never overwritten. Add --execute only to
perform the flight. Examples below include it.

Hover, system 2, five continuously settled seconds:

  python3 guided_swarm.py --target-systems 2 --pattern hover --height 1 \
    --hover 5 --execute --log /tmp/hover-02-01.jsonl

One movement and return, systems 1 and 2:

  python3 guided_swarm.py --target-systems 1 2 --pattern backward-return \
    --distance 1 --height 1 --hover 3 --hold 2 \
    --execute --log /tmp/backward-pair-01.jsonl

Also supported: forward-return, left-return, right-return. Directions are
relative to each aircraft's heading captured before takeoff; every target is
relative to its fixed initial hover point. Height and heading stay fixed.
--distance sets the excursion. --hold is the continuous settled dwell at each
movement/return target. All members must settle before the next leg starts.

--pattern three-axis performs left/return, right/return, backward/return.
All three excursions are horizontal. Example:

  python3 guided_swarm.py --target-systems 1 2 --pattern three-axis \
    --distance 0.5 --hover 3 --hold 1 --max-flight-time 60 \
    --execute --log /tmp/three-axis-pair-01.jsonl

Circle, single aircraft (circle remains the default pattern):

  python3 guided_swarm.py --target-systems 2 --pattern circle \
    --radius 0.25 --duration 12 --height 1 --hover 3 --hold 2 \
    --execute --log /tmp/circle-02-01.jsonl

Diamond, matching the successful 0.75 m two-aircraft flight:

  python3 guided_swarm.py --target-systems 1 2 --pattern diamond \
    --radius 0.75 --duration 35 --height 1 --hover 3 --hold 2 \
    --max-flight-time 60 --tracking-tolerance 0.50 \
    --execute --log /tmp/diamond-pair-075-01.jsonl

Use any pattern with one through four IDs. Use --target-systems even for a
single aircraft. Hover-only
uses --hover for its dwell, not the old single-flight --hold convention.

Path geometry and timing
------------------------
The group translates through the same circle/diamond, preserving nominal
spacing; aircraft do not orbit each other. --radius is half the width/depth.
The hover spot is the forwardmost vertex, not the centre. Maximum excursion
is 2*radius behind that spot. A 0.75 m diamond is 1.50 m wide and 1.50 m deep.
Its vertices, relative to hover (forward, right), are:

  (0,0) -> (-0.75,+0.75) -> (-1.50,0) -> (-0.75,-0.75) -> (0,0)

A circle uses a quintic phase with zero requested endpoint velocity and
acceleration. A diamond uses four straight quintic segments, each lasting
one quarter of --duration, with zero requested velocity/acceleration at the
corners. Corners are timed: there is no separate corner arrival barrier or
extra dwell. The final return still requires settled arrival. Position plus
velocity feedforward plus yaw is sent at 5 Hz. Acceleration fields are ignored.

Circle limits: radius 0.05–0.40 m, duration 10–20 s.
Diamond limits: radius 0.05–1.0 m, duration 10–40 s.
Requested peak speed must not exceed 0.35 m/s. Actual speed can exceed this.
The tested 0.75 m / 35 s diamond requests 0.227 m/s peak. A 1 m / 40 s diamond
requests 0.265 m/s and needs a 2 m-wide/deep path per aircraft; it has been
simulated but exceeds the user's current physical test space.

Fixed directional moves send position/yaw targets at 5 Hz; aircraft position
control determines transit speed. --speed is only a setup request and is not
an effective speed cap for these moves. Radius/duration and path-tracking
limits do not govern fixed-target moves.

Arrival defaults:
  Hover and fixed movements: 0.30 m position error, 0.40 m/s speed,
    30 s move timeout plus dwell (the earlier relaxed arrival criteria).
  Circle/diamond initial/final hover: 0.15 m error, 0.15 m/s speed,
    8 s move timeout plus dwell (the tested swarm criteria).
Override with --position-tolerance, --settle-speed and --move-timeout.
--hover defaults to 2 s; --hold defaults to 2 s. Hover-only uses only --hover.

--max-flight-time defaults to 45 s and may be increased to 60 s. It counts
from the start of arming through final hover, including takeoff, settling
and recovery pauses; landing is additional. Use 60 for the 35 s diamond.
Slow settling or repeated pauses can still end the routine at that limit.

Checks, recovery and stopping
----------------------------
Each aircraft needs fresh telemetry, vertical estimate, attitude and ground
state before arming. Existing EKF origins are reused; synthetic origin defaults
are 47.397742, 8.545594, 0 m AMSL, not a real physical location. Specify all of
--lat/--lon/--amsl together to supply an explicit origin. Home is checked against
local ground height. --height is above home; it is not a direct rangefinder
altitude command. Takeoff overshoot remains visible in physical logs.

All aircraft confirm arming before any takeoff command. The shared arming
policy uses ACK or observed armed state, never force-arm or a blind retry.
An explicit arming rejection stops the sequence. An initial usable horizontal
estimate is required after climb before any hover/movement phase begins.

Telemetry-rate and speed setup requests retry once if the ACK is missing,
then warn and rely on actual readiness. A missing TAKEOFF ACK after 5 s can be
confirmed by fresh post-command telemetry showing a ground-to-TAKEOFF/IN_AIR
transition, armed GUIDED state and valid vertical state. TAKEOFF is not resent.
An explicit or late rejection still stops the sequence.

Only a transient loss of the relative-horizontal-position validity flag gets
up to one second to recover. Fixed-target phases hold their target and restart
the settled dwell. A timed circle/diamond pauses the whole group at the last
commanded phase with zero velocity feedforward, then resumes without jumping
ahead. Overlapping peer losses cannot extend a path pause beyond one second.
Other required EKF flags, finite telemetry, heartbeat, mode and pilot-takeover
checks remain active. Persistent loss ends the routine.

Ground/climb position freshness is 1.5 s. After takeoff completes, position
receipt freshness is 0.6 s, and estimated pairwise separation must stay in
1–2 m. Path tracking is limited to 0.50 m estimated 3-D error by default;
--tracking-tolerance overrides it. Received position can lag the aircraft.
A path sender gap longer than 0.5 s stops rather than skipping ahead. The
trace logs targets, recovery, timestamps, errors and phase changes.

Normal completion uses LAND, ground confirmation and normal disarm. Failure
or first Ctrl-C requests best-effort LAND for eligible script-controlled
members; it never force-disarms. QGC takeover of one aircraft is preserved,
while other eligible members receive LAND. During landing, formation monitoring
ends and landing/disarm monitoring takes over. After completion or abort the
QGC relay stays active, but script GCS heartbeats stop. Keep QGC connected;
Ctrl-C again closes the relay.

Validation
----------
  python3 -m unittest discover -s companion -p 'test_guided_swarm*.py'
  python3 -m flake8 companion/guided_swarm.py companion/test_guided_swarm*.py

Tests cover protocol isolation, setup/ACK recovery, preflight, all patterns
and vehicle counts, geometry, bounded recovery, arrival dwell, timeouts,
tracking/separation, takeover and disarm. Simulated four-drone operation is
not a four-aircraft radio/clearance validation.
