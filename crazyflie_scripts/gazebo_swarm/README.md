# Crazyflie swarm simulation: ArduCopter SITL + Gazebo + AP_SwarmMesh

A swarm of Crazyflie 2.1 Brushless vehicles in one Gazebo Harmonic world. Each vehicle is
its own ArduCopter SITL process (JSON backend, lock-step with Gazebo). The vehicles see
each other only through **AP_SwarmMesh**, using the SITL multicast backend (`P2P_TYPE 10`).
Each flies a mission script onboard, with a synchronized takeoff into its own altitude
lane and a synchronized landing:

| `--mission` | Script | What it shows |
|---|---|---|
| `hover` (default) | `lua/swarm_hover.lua` | the synchronized swarm lifecycle: every vehicle holds its pad |
| `follow` | `lua/swarm_follow.lua` | **leader-follower over AP_SwarmMesh**: vehicle 1 flies a circle; the others keep their starting offsets from it, using only its mesh position and velocity |

`run_swarm.py` generates the world and starts every process. It acts as the
orchestrator, but it never arms a vehicle, changes its mode or sends it a target.
It only synchronizes the start, using standard ArduPilot machinery:

1. **MAVLink TIMESYNC.** The orchestrator sends `TIMESYNC` (tc1 = 0, ts1 = host time)
   to each vehicle at 5 Hz. Stock ArduPilot answers with its own `micros64()` clock,
   the same clock as Lua's `millis()`. `swarm_clock.py` fits an offset and a drift
   through the fastest round trips. The drift absorbs Gazebo running slower than
   real time.
2. **A parameter.** Once every vehicle reports READY, the orchestrator picks one
   host instant 5 s ahead (`--lead`). It sets `CFSW_START` on every vehicle to that
   instant expressed in that vehicle's clock, and confirms each value from the
   `PARAM_VALUE` echo.
3. **Onboard Lua.** Each script arms and takes off when its `millis()` reaches
   `CFSW_START`. It starts its mission at `t0 = CFSW_START + --climb` and lands at
   `t0 + --duration`, the same instants on every vehicle.

A vehicle is READY when it has a healthy position, all of its peers are fresh on the mesh,
it is in GUIDED (selected while disarmed; Copter refuses GUIDED without a usable position)
and its pre-arm checks pass, all held for 2 s.

## Setup (once)

1. ArduCopter SITL, from the ArduPilot root:
   `./waf configure --board sitl && ./waf copter`
2. Gazebo Harmonic (gz-sim 8). On macOS: `brew tap osrf/simulation && brew install gz-harmonic`.
   On Ubuntu: the OSRF apt repository, `sudo apt install gz-harmonic`.
3. The ArduPilot Gazebo plugin. Only `ArduPilotPlugin` is built, which avoids the
   GStreamer dependency of the full project:
   ```sh
   git clone https://github.com/ArduPilot/ardupilot_gazebo ~/ardupilot_gazebo   # tested at 082a0fe
   export ARDUPILOT_GAZEBO_DIR=~/ardupilot_gazebo
   cmake -S crazyflie_scripts/gazebo_swarm/plugin -B build/gz_plugin -DCMAKE_BUILD_TYPE=RelWithDebInfo
   cmake --build build/gz_plugin -j
   ```
   `run_swarm.py` looks for the library in `build/gz_plugin`. Use `--plugin-dir`
   to point elsewhere.
4. The Crazyflie model (`SITL_Models-crazyflie-brushless`, kept outside this
   repository because its meshes are CC BY-NC-SA):
   `export CRAZYFLIE_SITL_MODELS_DIR=~/SITL_Models-crazyflie-brushless`
5. Python 3.9+ with `pymavlink`. Install `lupa` too to run the Lua tests.

## Run

```sh
# 4 vehicles on a 0.6 m grid, GPS profile, Gazebo with GUI
crazyflie_scripts/gazebo_swarm/run_swarm.py --count 4

# leader-follower: vehicle 1 flies a 1 m circle at 0.15 m/s, one lap
crazyflie_scripts/gazebo_swarm/run_swarm.py --count 4 --mission follow

# GPS-denied: optical flow + downward ToF (the launcher adds a gpu_lidar to each model)
crazyflie_scripts/gazebo_swarm/run_swarm.py --count 3 --layout circle --profile indoor --headless

# no Gazebo: SITL's built-in generic quad (not a Crazyflie) to test mesh, sync and Lua
crazyflie_scripts/gazebo_swarm/run_swarm.py --count 3 --physics sitl

# inspect what would run
crazyflie_scripts/gazebo_swarm/run_swarm.py --count 8 --layout circle --dry-run
```

The orchestrator prints one status line every 2 s, showing each vehicle's state, peers,
altitude, tracking error and clock-fit residual, along with every STATUSTEXT. It exits 0
when every vehicle has landed. It writes `launch.json` and `summary.json` to the run
directory (default: a new temporary directory, or `--run-dir`). `summary.json` holds:

- the clock fit each start was scheduled with;
- the takeoff spread in host time;
- each vehicle's takeoff lateness against its own start time;
- each vehicle's tracking error: its distance from the mission's reference, mean and max.

Per-vehicle SITL logs and DataFlash logs are in `agent_<i>/`.

Key options: `--count`, `--mission hover|follow`, `--layout grid|circle`, `--spacing`,
`--radius`, `--profile gps|indoor`, `--alt` (centre lane), `--lanes`, `--lane-spacing`,
`--climb`, `--duration`, `--lead`, `--circle-radius` and `--circle-speed` (follow),
`--home=LAT,LON,ALT` (the world origin), `--headless`, `--extra-param-file`.

## Writing a mission

A mission script says where the vehicle should be. `lua/modules/swarm_common.lua` does
the rest of the flight: readiness, the synchronized start, the climb into the altitude
lane, tracking, the separation guard, and the landing. A mission returns a *reference*:
a world north/east position (m) and the velocity of that point (m/s), or `nil` to hold.
`swarm_common` flies toward it with `velocity + CFSW_KP * position error`, limited to
`CFSW_SPEED`. The whole hover mission is:

```lua
local swarm_common = require("swarm_common")

return swarm_common.run({
   name = "hover",
   reference = function(ctx, now)
      return ctx.site.pad_n, ctx.site.pad_e, 0, 0
   end,
})
```

`reference(ctx, now)` gets:

| | |
|---|---|
| `now` | this vehicle's `millis()` |
| `ctx.t0` | the common mission start in this vehicle's clock: use `now - ctx.t0` as mission time |
| `ctx.site` | this vehicle's generated `swarm_site.lua`: `index`, `sysid`, `count`, `sysids`, `pad_n`/`pad_e`, `pads[sysid]`, `leader_sysid`, `circle_radius`, `circle_speed`, timing, lanes |
| `ctx.peers` | `sysid -> {n, e, vn, ve}` for every peer AP_SwarmMesh has a fresh position for, in the world frame |
| `ctx.land(now, why)` | end the mission early |

`ctx.peers` is the mesh: each tick, `swarm_common` fills it from
`swarm:get_peer_location(sysid)` and `swarm:get_peer_velocity_NED(sysid)`. A peer drops
out of it when AP_SwarmMesh no longer has a fresh position for it.

### The follow example

`lua/swarm_follow.lua` is the leader-follower demo. Its follower is the whole idea:

```lua
local leader = ctx.peers[LEADER]
if leader == nil then
   return nil                                 -- hold until the leader is heard again
end
return leader.n + SLOT_N, leader.e + SLOT_E, leader.vn, leader.ve
```

Each follower's slot (`SLOT_N`, `SLOT_E`) is its pad minus the leader's pad, so the
formation is the starting layout. Feeding the leader's velocity forward lets the
followers move with the leader instead of trailing it. A follower that cannot hear the
leader holds its position, and lands after 5 s. The leader's reference is a point moving
around a circle that starts at its own pad, with the centre `--circle-radius` south of
it, at `--circle-speed`. The circle starts at `ctx.t0`, so the leader waits over its pad
until every vehicle has finished climbing.

To add a mission: write `lua/swarm_<name>.lua` and add it to `MISSIONS` in
`run_swarm.py`. Add any data it needs to `site_lua()`.

### Using swarm_hub.py or a GCS instead

`--no-monitor` leaves each vehicle's MAVLink UDP port free. The mission script then waits
in READY, because nothing sets `CFSW_START`. In this mode, connect
`companion/swarm_hub.py --sim 5760 --sim 5770 ...` or a GCS.

## Ports and frames

| Vehicle *i* (0-based) | |
|---|---|
| SITL instance / `MAV_SYSID` | `-I i` / `i + 1` |
| MAVLink (serial0, `udpclient`) | UDP `5760 + 10 i` |
| JSON FDM to the Gazebo plugin | UDP `9002 + 10 i` |
| AP_SwarmMesh | multicast `239.65.83.0`: **one swarm per host** (the launcher holds a lock) |

Serial0 is `udpclient`, not TCP. A TCP serial port makes SITL wait for a client, and in
lock-step that would stall the whole Gazebo world.

World frame: Gazebo `(x, y)` is (east, north). Every vehicle uses the world origin
(`--home`) as its SITL home, and Gazebo places it on its pad. With `--physics sitl`, each
vehicle's home is its pad instead. With `--profile indoor` the script sets the EKF origin
to its pad, so every vehicle's position, and what AP_SwarmMesh broadcasts, is in the one
world frame.

## Parameters

`--defaults` order: `Tools/autotest/default_params/copter.parm`, then
`../crazyflie_sitl_gazebo.parm` (airframe and tune; Gazebo only), `params/common.parm`,
`params/profile_<gps|indoor>.parm`, `params/swarmmesh_sitl.parm`,
`params/sitl_physics_indoor.parm` (SITL physics with the indoor profile only),
`--extra-param-file`s, and finally the per-vehicle `agent.parm`.

The launcher rewrites the model's `no_time_sync` to 0. With 1, `SIM_JSON` silently selects
`AHRS_EKF_TYPE 10` (perfect state) instead of EKF3. The orchestrator checks
`AHRS_EKF_TYPE == 3` and `P2P_TYPE == 10` before scheduling a start.

Mission parameters, created by `swarm_common.lua`: `CFSW_START`, `CFSW_SPEED`,
`CFSW_KP`, `CFSW_AVOID`, `CFSW_PUSH`, `CFSW_TOL` and `CFSW_VZMAX` (see its header). The
separation guard is a simple bounded repulsion inside `CFSW_AVOID`. It is a safety net
for the demos, not a collision-avoidance method.

## Tests

```sh
python3 -m unittest discover -s crazyflie_scripts/gazebo_swarm/tests
```

These tests cover file generation, the clock fit, the shared mission state machine and
both missions (Lua, with stubbed bindings via `lupa`). None of them needs Gazebo or SITL.

## Troubleshooting

- Every vehicle stays at `p0` (no peers): the SITL mesh runs on loopback, so check that
  the ArduPilot build includes `AP_SwarmMesh: keep the SITL mesh on loopback`. Before that
  change, the mesh used the default interface, which macOS refuses for multicast sends
  (`EPIPE`) on some setups, as do VPN default routes.
- *ports already in use* / *another swarm simulation is running*: another SITL or
  swarm is still running. Stop it; two swarms on one host would hear each other's mesh.
- A vehicle stays in WAIT: its STATUSTEXT `swarm: waiting: <reason>` says why (peers, no
  position, GUIDED not accepted yet, pre-arm checks).
- Gazebo starts but the vehicles never boot: check that `gazebo.log` shows the plugin
  loaded, and that it was built against the same Gazebo as `gz`.
- macOS: the Gazebo server and GUI run as separate processes (`-s`, `-g`), which the
  launcher handles. `GZ_IP=127.0.0.1` avoids discovery problems on Wi-Fi.
