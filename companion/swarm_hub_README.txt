swarm_hub.py -- radio links, QGC relay and GUIDED missions in one process
=========================================================================

Start it once; QGC stays connected and missions are flown from its prompt.
Run from ardupilot/ with a venv (pymavlink + pyusb):

  PY=../../../nrf51/nrf-firmware-cus/.venv/bin/python

  # 1 drone
  $PY companion/swarm_hub.py --uri radio://0/80/2M/E7E7E7E701

  # 2 drones, one dongle
  $PY companion/swarm_hub.py --uri radio://0/80/2M/E7E7E7E701 --uri radio://0/80/2M/E7E7E7E702

  # 3 drones, two dongles
  $PY companion/swarm_hub.py --uri radio://0/80/2M/E7E7E7E701 --uri radio://0/80/2M/E7E7E7E702 --uri radio://1/100/2M/E7E7E7E703

  # 4 drones, two dongles
  $PY companion/swarm_hub.py --uri radio://0/80/2M/E7E7E7E701 --uri radio://0/80/2M/E7E7E7E702 --uri radio://1/100/2M/E7E7E7E703 --uri radio://1/100/2M/E7E7E7E704

URI = radio://<dongle>/<channel>/2M/<address>. Drones on one dongle share its channel; the two dongles must use different channels. Only list drones that are powered. Each drone needs its own MAV_SYSID. QGC listens on UDP 14550.

Prompt commands
---------------
  fly <options>      show the plan, ask y/N, fly
  preview <options>  plan only
  again              repeat the last fly (asks y/N)
  status             links, armed state, dongles, current log
  help fly           all mission options
  quit               or Ctrl-D

All drones heard are flown unless --target-systems is given. Ctrl-C during a mission aborts it (LAND, never a forced disarm) and returns to the prompt.

Missions
--------
  Hover:
    fly --pattern hover --height 1 --hover 5

  Out and back (forward-return, backward-return, left-return, right-return):
    fly --pattern left-return --distance 0.5 --hover 3 --hold 2

  Left, centre, right, centre, backward, centre, land:
    fly --pattern three-axis --distance 0.5 --hover 3 --hold 1 --max-flight-time 60

  Circle (radius 0.05-0.4 m, duration 10-20 s):
    fly --pattern circle --radius 0.25 --duration 12 --hover 3 --hold 2

  Diamond (radius 0.05-1.0 m, duration 10-40 s; 0.75 m is 1.5 m wide/deep):
    fly --pattern diamond --radius 0.75 --duration 35 --hover 3 --hold 2 --max-flight-time 60

Note:
--height is metres above home.
--hover is the settled hover before moving.
--hold the settled dwell time at each target.
--distance applies to every leg and must exceed position tolerence.
--position-tolerance (Default = 0.30 m) is the target position error tolerence.
--settle-speed (Default = 0.15) is the speed below which a drone counts as stopped at a target.
--max-flight-time (default = 45 s, max 60) runs from arming to the final hover; landing is extra.
Paths are limited to 0.35 m/s requested peak speed.

Placement
---------
All drones must face the same direction on level ground. Recommended drone placement:
  1: centre
  2: side by side (1.5 m apart)  
  3: triangle (1.5 m apart, first in front)
  4: square (1.5 m apart, first two in front)          --spacing changes these

Logs
----
companion/logs/<date>/, named automatically: a session .tlog (all MAVLink), a session .jsonl (link events) and one .jsonl per mission.

Tests: python3 -m unittest discover -s companion -p 'test_*.py'

OptiTrack (Motive)
------------------
The hub can forward Motive rigid bodies to their drones as ATT_POS_MOCAP (ArduPilot's
ExternalNav input). Each rigid body is mapped to one MAV_SYSID and only that drone gets it.
Needs the NatNet client in the same venv:  $PY -m pip install natnet

  # 2 drones, one dongle; Motive rigid bodies 66 and 67 fly as systems 1 and 2
  $PY companion/swarm_hub.py --uri radio://0/80/2M/E7E7E7E701 --uri radio://0/80/2M/E7E7E7E702 \
      --mocap-server 192.168.50.208 --mocap-local 192.168.50.198 \
      --mocap-body 66:1 --mocap-body 67:2

  --mocap-body RB:SYSID   repeatable; each rigid body and each system used once
  --mocap-server IP       the Motive PC
  --mocap-local IP        this computer's address on the Motive network
  --mocap-unicast         if Motive streams unicast (default multicast)
  --mocap-frame y-up      Motive's up axis: y-up (default: N = -Z, E = X, D = -Y)
                          or z-up (N = X, E = -Y, D = -Z)
  --mocap-rate 30         poses per second per drone (0 = every Motive frame)
  --mocap-max-queue 2     skip a pose while that drone's radio uplink is backed up

Motive setup: stream rigid bodies (NatNet) on the network the hub uses. Create each
rigid body with the drone level and its nose along the axis that maps to north
(-Z for y-up, +X for z-up), so the body axes start aligned with FRD.

A pose goes out only once its drone has been heard (heartbeat with that sysid), while
Motive reports the body tracked, and while that drone's uplink queue is short; the
radio sends one frame per poll, so stale poses are skipped rather than queued.
"status" shows each body's state, send rate and last NED position; the session log
gets mocap_stats every --stats-sec, and every pose sent is in the session .tlog.

Timing
  Each pose is stamped at its capture: the NatNet arrival minus Motive's latency (measured
  from the frame's camera and transmit stamps once their tick rate is known, about 2 s
  after start; 5 ms until then; --mocap-latency-ms N fixes it).
  --mocap-time hub   (default) time_usec in the hub's clock; ArduPilot's stock jitter
                     correction absorbs the offset. For flying on mocap (ExternalNav).
  --mocap-time fc    time_usec in each drone's own clock: the hub sends it TIMESYNC at 2 Hz
                     (stock ArduPilot answers) and fits the offset and drift through the
                     fastest round trips. A drone gets no poses until about 10 answers are
                     in (5 s). For the cooperative estimator: AP_CoopBridge with
                     COOP_FIX_SRC 1, COOP_FIX_TIME 1 uses time_usec as the capture time.
  "status" shows per drone the shortest round trip, the fit residual and the clock drift.
  Keep --mocap-rate >= 30 with COOP_FIX_HZ: the bridge forwards the first pose captured
  after each due 100 ms knot, so poses must arrive often enough to land early in the knot.

Ground truth
  Every Motive frame (all rigid bodies, full rate, mapped or not, tracked or not) goes to
  <log dir>/<date>/HHMMSS-mocap-truth.csv: hub time, capture time (hub and, once synced,
  the drone's clock), Motive frame and timestamp, latency, raw Motive pose and NED pose.
  --no-mocap-truth turns it off.

Cooperative estimator (thesis, P5 global fixes) instead of ExternalNav:
  hub: --mocap-time fc
  FC:  COOP_FIX_SRC 1, COOP_FIX_TIME 1, COOP_FIX_HZ 1 (0 on drones without fixes),
       COOP_FIX_DOF 6/4/3, COOP_FIX_P_SD, COOP_FIX_R_SD; keep VISO_TYPE 1 and EK3_SRC1_* = 6
       for the companion's ODOMETRY.

ArduPilot parameters (flying on mocap through ExternalNav; check against your setup):
  VISO_TYPE      1   (MAVLink)
  VISO_DELAY_MS  ~30 (Motive + radio latency; measure)
  VISO_POS_M_NSE 0.05, VISO_YAW_M_NSE 0.05
  EK3_SRC1_POSXY 6, EK3_SRC1_POSZ 6 (or 2 to keep the rangefinder),
  EK3_SRC1_VELXY 0 (or 5 to keep optical flow), EK3_SRC1_VELZ 0, EK3_SRC1_YAW 6
  The mocap frame's north is not true north: do not mix compass yaw with ExternalNav position.
