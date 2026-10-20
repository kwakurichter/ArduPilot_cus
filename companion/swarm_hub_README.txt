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
