Crazyflie GUIDED flight test over the existing Crazyradio bridge
==============================================================

Sequence: set/reuse EKF origin; set home altitude to the current ground height;
enter GUIDED; normal arm; take off to 1 m above home; settle; move 1 m left;
settle; move 1 m right back to the starting position; settle; LAND; wait for
confirmed touchdown and normal disarm. Left is relative to heading captured
before takeoff. Heading is held during the movement targets.

This is a host-side Python script. It requires no new flight-controller
firmware and does not change PID gains or disable arming checks. The altitude,
distance, speed and dwell can be changed with command-line options.

Three-axis flight pattern
-------------------------
Use --pattern three-axis for:
  takeoff -> settle -> 1 m left -> start -> 1 m right -> start
          -> 1 m backward -> start -> LAND -> confirmed normal disarm.
All directions use the heading captured before takeoff. Each target is an
absolute offset from the same initial hover target, not from the current
estimated position or accumulated leg endpoints. Altitude and heading stay
fixed for the translation targets. The defaults remain 1 m excursions, 0.30 m
3D arrival radius, 0.40 m/s arrival speed threshold, 1 s continuous settled dwell
and 30 s timeout per leg.
--pattern left-return remains the default for the original short mission.

With the existing bridge/relay setup, preview from the isolated clone root:
  python3 companion/guided_flight.py --target-system 1 --pattern three-axis

To execute the longer pattern:
  python3 companion/guided_flight.py --target-system 1 --pattern three-axis --execute --log /tmp/guided-three-axis-01.jsonl

After landing, the relay remains active for QGC. Close it with Ctrl-C before
starting another script instance. Use a fresh log name for each run. Keep the
bridge pointed to 127.0.0.1:14560; QGC listens at 14550.

Keep surface, lighting, battery state and measured flight height comparable.
The pattern is about 6 m of nominal horizontal travel, plus takeoff, landing
and corrections. The per-leg timeout can stop the mission and request LAND.
The sequence has completed in simulation and user-operated hardware flights;
this does not establish position accuracy or a fully tuned controller.

Files
-----
guided_flight.py         Flight sequencer with an integrated UDP relay.
test_guided_flight.py    Protocol and abort-path unit tests.

Requires Python 3.9+ and pymavlink in the Python environment used to run it:
    python3 -m pip install pymavlink

Connection setup
----------------
The supplied mavlink_bridge.py has ONE UDP peer, replaced by each sender.
Do not connect QGC and the sequencer independently to that bridge port.
The integrated relay keeps the bridge peer fixed and forwards both ways:

    Crazyradio bridge <-> script listener 127.0.0.1:14560
                                  <-> QGC UDP listener 127.0.0.1:14550

1. Keep QGC listening on its normal UDP port 14550.
2. Restart the existing bridge, preserving your URI and other options, but
   change its destination to --udp 127.0.0.1:14560. For the default radio URI:

    python3 /Users/kwaku/Documents/Crazyflie/nrf51/nrf-firmware-cus/tools/mavlink_bridge.py --udp 127.0.0.1:14560

   Use the same Python environment you normally use for that bridge.
3. From this repository's root, preview the sequence:

    python3 companion/guided_flight.py --target-system 1

   Preview opens no sockets and sends no commands. QGC will receive telemetry
   through this route once the script is started with --execute.
4. Start on the ground, disarmed. To execute the sequence:

    python3 companion/guided_flight.py --target-system 1 --execute --log /tmp/guided-flight-01.jsonl

   --target-system must match the aircraft's MAVLink system ID. The default
   source ID is 255; --source-system must match the permitted GCS ID if your
   firmware filters GCS commands. Script component ID is 191; QGC normally
   uses 190. A pre-existing log file is never overwritten: choose a new name.

The bridge must receive telemetry within 20 seconds. Do not run two copies of
this sequencer, or a second controller issuing GUIDED targets, at once.

Origin and home
---------------
No latitude/longitude is required for this local indoor experiment. The script
first requests GPS_GLOBAL_ORIGIN. It reuses an existing origin. If none exists,
it requests the synthetic origin 47.397742, 8.545594, 0 m AMSL using
COMMAND_INT / MAV_CMD_DO_SET_GLOBAL_ORIGIN (611), checks its ACK and reads back
GPS_GLOBAL_ORIGIN. Command altitude is in metres; readback altitude is in mm. These are arbitrary coordinates, NOT the aircraft's location.
Use this default only for local non-GPS tests; it is not a real geographic
reference for GPS, maps, geofences or global-coordinate missions.

For a real geographic reference supply all three options:
    --lat LATITUDE --lon LONGITUDE --amsl GROUND_ALTITUDE_METRES
An existing origin more than 2 m away from explicitly requested coordinates
causes the script to stop. It never tries to overwrite an established origin.

EKF origin and home are different. Once the vertical estimate is ready, an
explicit MAV_CMD_DO_SET_HOME (COMMAND_INT, param1=0) locks home at the origin
latitude/longitude and the current ground altitude (origin AMSL minus local NED
z). This avoids the valid horizontal location required by "use current home".
The 1 m takeoff command is relative to HOME. Movement targets are local NED
positions computed using the origin/home altitude difference, not z=-1 blindly.

Known height-reference limitation (2026-10-06)
----------------------------------------------
The script captures the EKF local ground height before arming to set home.
Logs 22 and 25 recorded home altitudes of 0.07 and 0.16 m, respectively, and
local movement-height targets of 1.07 and 1.16 m for the nominal 1 m mission.
Measured ground clearance differed more (about 1.10 versus 1.34 m). Altitude
tracked its local target closely; MOT_THST_HOVER stayed at 0.474 in both logs.
The startup home/local reference and EKF height-to-ground offset remain under
investigation. A nominal --height is relative to the captured home; it is not
an independently verified physical clearance. No correction is included yet.

Movement and completion
-----------------------
The pre-takeoff MAV_CMD_DO_CHANGE_SPEED request remains 0.2 m/s. In this firmware,
entering position control resets that setting to the firmware default; it is not
an effective limit for these position legs. This behavior is preserved to retain
the movement speed preferred in log_12. --settle-speed only controls arrival
detection and does not change vehicle speed.
Targets are fixed local positions, resent at 5 Hz with a fixed yaw. Resends do
not accumulate another 1 m offset. For the current untuned flight tests, arrival
requires 3D position error <= 0.30 m and 3D speed <= 0.40 m/s continuously for
1 second. These are coarse waypoint acceptance limits, not a controller-tuning
performance requirement. Any excursion resets the dwell timer; a fast pass
through the target does not count. Progress is printed and logged once per
second, showing error, speed and accumulated dwell. Each move times out after
30 s. Tighten --position-tolerance, --settle-speed and --hold after tuning.
In the default left-return pattern, the rightward leg returns to the starting
point. The three-axis pattern adds separate right and backward excursions,
each followed by a return to that same starting point.

Options: --height, --distance, --speed, --hold, --position-tolerance,
--settle-speed, --move-timeout, --listen, --qgc. Run --help for details.

Normal arming is retried within an 8 s window only after an explicit temporary
rejection or FAILED result. A command ACK can take up to 5 s. A lost arming ACK
is checked against the armed heartbeat, never blindly retried. No force-arm is
sent and the autopilot still performs its own arming checks.

The script requests LOCAL_POSITION_NED at 10 Hz, attitude at 5 Hz, and heartbeat,
EKF status, system health and landed-state telemetry at 2 Hz. These change link stream rates;
they are not restored on exit. Commands, targets and received telemetry are
saved in the JSONL log for comparison with the onboard flight log.

Takeover and failure behavior
-----------------------------
Changing flight mode stops further trajectory commands. QGC mode/land/arm and
setpoint commands also cause the sequencer to yield, while forwarding that
command. After takeover it does not automatically command LAND over the pilot.
Normal joystick input is forwarded; use a mode change to explicitly take over.

The first Ctrl-C during a flight stops the sequence and, if the last fresh
heartbeat still reports GUIDED and no takeover was observed, sends one
best-effort LAND. A telemetry/command/settling failure follows the same policy.
A lost link cannot guarantee delivery of LAND. No force-disarm is ever sent.

After normal landing, disarm is sent only after fresh EXTENDED_SYS_STATE has
reported ON_GROUND continuously for 2 s. Autopilot auto-disarm is also accepted.
If touchdown cannot be confirmed, no disarm command is sent.

After completion, takeover or error, the relay stays running so QGC retains the
radio connection. The script stops its own GCS heartbeats; QGC's packets keep
passing through. Ctrl-C at this point closes the relay. To reconnect QGC without
this script, restart the bridge with its original --udp 127.0.0.1:14550 setting.

Validation
----------
Run the protocol and sequence tests from the repository root:
  python3 -m unittest discover -s companion -p test_guided_flight.py

AI-assisted code and tests. Simulation tests do not establish physical flight
performance.

Ground-level position readiness on this aircraft
-----------------------------------------------
This custom Crazyflie firmware defines HAL_CF21 and exempts GUIDED from requiring
horizontal position for arming. Below-minimum ground range can prevent a valid
horizontal-position estimate; setting an origin alone does not make it valid.

Before takeoff the script requires an initialized EKF with attitude, vertical
velocity and vertical position flags, plus fresh finite LOCAL_POSITION_NED,
attitude and ON_GROUND telemetry. Horizontal EKF validity is checked after the
climb and throughout all position-target movements. If it is still unavailable
at takeoff height, the sequence stops and requests LAND under the abort policy.
If LOCAL_POSITION_NED is not published at all on the ground, this script will
still stop; it does not estimate launch height from missing telemetry.

The companion code also contains a timed STABILIZE/force-disarm fallback during
landing. This script does not copy that behavior: it uses confirmed ground state
and normal disarm, or accepts autopilot auto-disarm.
