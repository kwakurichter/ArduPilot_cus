#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Logic tests of the mission scripts and swarm_common.lua with stubbed bindings (needs lupa).

    python3 -m unittest discover -s crazyflie_scripts/gazebo_swarm/tests
"""

import unittest
from pathlib import Path

try:
    import lupa
except ImportError:  # pragma: no cover
    lupa = None

# velocity tolerance: a stub Location, like a real one, holds 1e-7 degrees (~1.1 cm),
# which the position gain (0.8 /s) turns into up to ~0.01 m/s
Q = 0.012

LUA_DIR = Path(__file__).resolve().parents[1] / "lua"

# Minimal stand-ins for the bindings the script uses. Positions are flat-earth
# metres from a fixed origin; a Location stores lat/lng in 1e-7 degrees.
STUBS = r"""
local M_PER_E7 = 0.011131884502145034   -- metres per 1e-7 degree of latitude
sim = {now = 0, params = {}, texts = {}, named = {}, mode = 0, armed = false, healthy = true,
       position = nil, alt = 0, origin = nil, peers = {}, prearm = true, guided_ok = true,
       arm_calls = {}, takeoff = nil, velocity = nil, modes = {}, set_origin_calls = 0,
       peer_vel = {}}

local Loc = {}
Loc.__index = Loc
function Location() return setmetatable({_lat = 0, _lng = 0, _alt = 0}, Loc) end
function Loc:lat(v) if v then self._lat = v end return self._lat end
function Loc:lng(v) if v then self._lng = v end return self._lng end
function Loc:alt(v) if v then self._alt = v end return self._alt end
function Loc:offset(n, e)
   self._lat = self._lat + math.floor(n / M_PER_E7 + 0.5)
   self._lng = self._lng + math.floor(e / M_PER_E7 + 0.5)   -- equator scaling is fine here
end
function Loc:get_distance_NE(other)
   return Vector2f_new((other._lat - self._lat) * M_PER_E7, (other._lng - self._lng) * M_PER_E7)
end

local V2 = {}
V2.__index = V2
function Vector2f_new(x, y) return setmetatable({_x = x, _y = y}, V2) end
function V2:x() return self._x end
function V2:y() return self._y end

local V3 = {}
V3.__index = V3
function Vector3f() return setmetatable({_x = 0, _y = 0, _z = 0}, V3) end
function V3:x(v) if v then self._x = v end return self._x end
function V3:y(v) if v then self._y = v end return self._y end
function V3:z(v) if v then self._z = v end return self._z end

function at_ne(n, e)       -- a Location n/e metres from the stub world origin (0, 0)
   local l = Location()
   l:offset(n, e)
   return l
end

local P = {}
P.__index = P
function Parameter(name) return setmetatable({name = name}, P) end
function P:get() return sim.params[self.name] end
param = {}
function param:add_table(key, prefix, n) sim.prefix = prefix return true end
function param:add_param(key, idx, name, default) sim.params[sim.prefix .. name] = default return true end
function param:get(name) return sim.params[name] end

millis_obj = {}
function millis() return {toint = function() return sim.now end} end

gcs = {}
function gcs:send_text(sev, text) table.insert(sim.texts, text) end
function gcs:send_named_float(name, value) sim.named[name] = value end

ahrs = {}
function ahrs:healthy() return sim.healthy end
function ahrs:initialised() return true end
function ahrs:get_location() return sim.position end
function ahrs:get_origin() return sim.origin end
function ahrs:set_origin(loc) sim.origin = loc sim.set_origin_calls = sim.set_origin_calls + 1 return true end
function ahrs:get_relative_position_D_home() return -sim.alt end

swarm = {}
function swarm:get_peer_location(sysid) return sim.peers[sysid] end
function swarm:get_peer_velocity_NED(sysid)
   local v = sim.peer_vel[sysid]
   if v == nil then return nil end
   local out = Vector3f()
   out:x(v[1]) out:y(v[2]) out:z(0)
   return out
end

vehicle = {}
function vehicle:get_mode() return sim.mode end
function vehicle:set_mode(m)
   table.insert(sim.modes, m)
   if m == 4 and not sim.guided_ok then return false end
   sim.mode = m
   return true
end
function vehicle:start_takeoff(alt) sim.takeoff = alt return true end
function vehicle:set_target_velocity_NED(v) sim.velocity = {v:x(), v:y(), v:z()} return true end

arming = {}
function arming:is_armed() return sim.armed end
function arming:pre_arm_checks() return sim.prearm end
function arming:arm()
   table.insert(sim.arm_calls, sim.now)
   if sim.prearm then sim.armed = true end
   return sim.armed
end
"""


def site_table(index=0, count=3, profile="gps", climb=10.0, duration=20.0, mission="hover"):
    pads = [(0.0, -0.6), (0.0, 0.0), (0.0, 0.6)][:count]
    rows = ", ".join(f"{{n = {n}, e = {e}}}" for n, e in pads)
    sysids = ", ".join(str(i + 1) for i in range(count))
    return (f"{{index = {index}, count = {count}, sysid = {index + 1}, sysids = {{{sysids}}}, "
            f"origin_lat_e7 = 0, origin_lng_e7 = 0, origin_alt_cm = 0, "
            f"pad_n = {pads[index][0]}, pad_e = {pads[index][1]}, pads = {{{rows}}}, "
            f"profile = \"{profile}\", mission = \"{mission}\", alt = 1.0, lane_count = 4, "
            f"lane_spacing = 0.25, climb_s = {climb}, duration = {duration}, "
            f"leader_sysid = 1, circle_radius = 1.0, circle_speed = 0.15}}")


@unittest.skipIf(lupa is None, "lupa is not installed")
class MissionScript(unittest.TestCase):

    SCRIPT = "swarm_hover.lua"

    def load(self, **site):
        site.setdefault("mission", self.SCRIPT[len("swarm_"):-len(".lua")])
        self.lua = lupa.LuaRuntime(unpack_returned_tuples=True)
        self.lua.execute(STUBS)
        self.lua.globals().package.path = str(LUA_DIR / "modules" / "?.lua")
        self.lua.execute(f"package.preload['swarm_site'] = function() return {site_table(**site)} end")
        self.sim = self.lua.globals().sim
        self.index = site.get("index", 0)
        self.count = site.get("count", 3)
        self.update, self.next_ms = self.lua.execute((LUA_DIR / self.SCRIPT).read_text())

    def place(self, n, e, alt=0.0):
        self.sim.position = self.lua.globals().at_ne(n, e)
        self.sim.alt = alt

    def peers_at(self, positions):
        peers = self.lua.table()
        for sysid, (n, e) in positions.items():
            peers[sysid] = self.lua.globals().at_ne(n, e)
        self.sim.peers = peers

    def all_peers_on_pads(self):
        pads = {1: (0.0, -0.6), 2: (0.0, 0.0), 3: (0.0, 0.6)}
        self.peers_at({s: p for s, p in pads.items() if s != self.index + 1 and s <= self.count})

    def run_until(self, t_ms, hook=None):
        """Run the script's scheduler until the stub clock reaches t_ms."""
        while self.sim.now + self.next_ms <= t_ms:
            self.sim.now = self.sim.now + self.next_ms
            if hook:
                hook()
            result = self.update()
            self.assertIsNotNone(result, f"script stopped: {list(self.sim.texts.values())}")
            self.update, self.next_ms = result
        self.sim.now = t_ms

    STATES = ["SITE", "WAIT", "READY", "TAKEOFF", "FLY", "LAND", "DONE", "ABORT"]

    def state(self):
        """Current state, from the script's latest state-change message (SWSTATE is rate limited)."""
        for text in reversed(self.texts()):
            word = text.split(" ")[1] if text.startswith("swarm: ") else ""
            if word in self.STATES:
                return self.STATES.index(word)
        return 0

    def texts(self):
        return list(self.sim.texts.values())

    def ready_vehicle(self, **site):
        self.load(**site)
        pad_e = [-0.6, 0.0, 0.6][self.index]
        self.place(0.0, pad_e)
        self.all_peers_on_pads()
        self.run_until(5000)
        self.assertEqual(self.state(), 2, self.texts())   # READY

    def test_ready_then_synchronized_takeoff_hover_land(self):
        self.ready_vehicle(index=1)
        self.assertEqual(self.sim.mode, 4, "GUIDED is selected while disarmed")
        self.sim.params["CFSW_START"] = 7321.0
        self.run_until(7400)
        self.assertEqual(list(self.sim.arm_calls.values()), [7321], "arms exactly at CFSW_START")
        # lanes for 3 vehicles: 0.75, 1.0, 1.25 m; vehicle index 1 -> 1.0 m
        self.assertAlmostEqual(self.sim.takeoff, 1.0)
        self.assertEqual(self.state(), 3)   # TAKEOFF
        self.sim.alt = 0.95
        self.run_until(8000)
        self.assertEqual(self.state(), 4)   # FLY
        # drifted 0.2 m north of the pad: commanded back south, and down a little
        self.place(0.2, 0.0, 1.1)
        self.run_until(8200)
        vn, ve, vd = self.sim.velocity.values()
        self.assertLess(vn, 0)
        self.assertAlmostEqual(ve, 0, places=3)
        self.assertGreater(vd, 0)
        # land at CFSW_START + climb + duration = 7321 + 30000
        self.run_until(37300)
        self.assertEqual(self.state(), 4)
        self.run_until(37500)
        self.assertEqual(self.sim.mode, 9)
        self.sim.armed = False
        self.run_until(38000)
        self.assertEqual(self.state(), 6)   # DONE

    def test_waits_for_peers(self):
        self.load(index=0)
        self.place(0.0, -0.6)
        self.peers_at({2: (0.0, 0.0)})      # vehicle 3 missing
        self.run_until(8000)
        self.assertEqual(self.state(), 1)   # WAIT
        self.assertIn("swarm: waiting: 1 of 2 peers", self.texts())

    def test_guided_refused_keeps_waiting(self):
        self.load(index=0)
        self.sim.guided_ok = False
        self.place(0.0, -0.6)
        self.all_peers_on_pads()
        self.run_until(8000)
        self.assertEqual(self.state(), 1)
        self.assertIn("swarm: waiting: GUIDED not accepted yet", self.texts())

    def test_late_start_is_refused(self):
        self.ready_vehicle(index=0)
        self.sim.params["CFSW_START"] = float(self.sim.now - 2000)
        self.run_until(self.sim.now + 300)
        self.assertEqual(self.state(), 7)   # ABORT
        self.assertEqual(len(self.sim.arm_calls), 0)

    def test_brief_prearm_flicker_keeps_ready(self):
        self.ready_vehicle(index=0)
        self.sim.prearm = False
        self.run_until(self.sim.now + 500)
        self.assertEqual(self.state(), 2)
        self.sim.prearm = True
        self.sim.params["CFSW_START"] = float(self.sim.now + 1000)
        self.run_until(self.sim.now + 1200)
        self.assertEqual(self.state(), 3)

    def test_arming_retried_within_grace_then_abort(self):
        self.ready_vehicle(index=0)
        start = self.sim.now + 1000
        self.sim.params["CFSW_START"] = float(start)

        def flicker():
            # not ready exactly at the start time, recovered 200 ms later
            self.sim.prearm = not (start <= self.sim.now < start + 200)
        self.run_until(start + 400, flicker)
        self.assertEqual(self.state(), 3)
        self.assertGreaterEqual(min(self.sim.arm_calls.values()), start + 200)

        self.ready_vehicle(index=0)
        start = self.sim.now + 1000
        self.sim.params["CFSW_START"] = float(start)
        self.run_until(start - 1)
        self.sim.prearm = False
        self.run_until(start + 1500)
        self.assertEqual(self.state(), 7)

    def test_separation_guard_pushes_away(self):
        self.ready_vehicle(index=1)
        self.sim.params["CFSW_START"] = float(self.sim.now + 100)
        self.run_until(self.sim.now + 200)
        self.sim.alt = 1.0
        self.run_until(self.sim.now + 200)
        self.assertEqual(self.state(), 4)
        # on its pad, with vehicle 1 only 0.2 m to the west
        self.peers_at({1: (0.0, -0.2), 3: (0.0, 0.6)})
        self.run_until(self.sim.now + 2000)
        vn, ve, vd = self.sim.velocity.values()
        self.assertGreater(ve, 0.1, "pushed east, away from the peer")
        self.assertLessEqual(ve, self.sim.params["CFSW_PUSH"] + 1e-6)

    def test_indoor_sets_origin_on_the_pad(self):
        self.load(index=2, profile="indoor")
        self.run_until(3000)
        self.assertEqual(self.sim.set_origin_calls, 1)
        origin = self.sim.origin
        self.assertEqual(origin.lat(origin), 0)
        # within the 1e-7 degree (~1.1 cm) quantization of a Location
        self.assertAlmostEqual(origin.lng(origin) * 0.011131884502145034, 0.6, delta=0.012)

    def test_mission_must_match_site(self):
        with self.assertRaises(lupa.LuaError):
            self.load(index=0, mission="follow")

    def take_off_and_fly(self, index):
        """Ready, synchronized start, climb to the lane; returns the common mission start t0."""
        self.ready_vehicle(index=index)
        start = self.sim.now + 1000
        self.sim.params["CFSW_START"] = float(start)
        self.run_until(start + 100)
        self.sim.alt = [0.75, 1.0, 1.25][index]
        self.run_until(start + 300)
        self.assertEqual(self.state(), 4, self.texts())   # FLY
        return start + 10000

    def command(self):
        return list(self.sim.velocity.values())


@unittest.skipIf(lupa is None, "lupa is not installed")
class FollowScript(MissionScript):
    """swarm_follow.lua: pads (N, E) are (0, -0.6), (0, 0), (0, 0.6); vehicle 1 leads."""

    SCRIPT = "swarm_follow.lua"

    # the hover-specific checks of the parent class do not apply
    test_ready_then_synchronized_takeoff_hover_land = None
    test_separation_guard_pushes_away = None
    test_mission_must_match_site = None

    def test_leader_waits_over_its_pad_then_circles(self):
        t0 = self.take_off_and_fly(index=0)
        self.place(0.0, -0.6, 0.75)
        self.run_until(t0 - 500)
        vn, ve, _ = self.command()
        self.assertAlmostEqual(vn, 0, delta=Q)
        self.assertAlmostEqual(ve, 0, delta=Q)
        # a quarter lap after t0 the reference is east of the centre (-1, -0.6), moving south
        quarter = int(1000 * (3.14159265 / 2) / 0.15)
        self.run_until(t0 + quarter)
        self.place(-1.0, 0.4, 0.75)
        self.run_until(t0 + quarter + 100)
        vn, ve, _ = self.command()
        self.assertAlmostEqual(vn, -0.15, delta=Q)
        self.assertAlmostEqual(ve, 0.0, delta=Q)

    def test_follower_holds_its_offset_and_matches_leader_velocity(self):
        self.take_off_and_fly(index=2)
        # the leader has moved to (1, 2) at (0.1, -0.05) m/s; this vehicle's slot is +1.2 m east
        self.peers_at({1: (1.0, 2.0), 2: (0.0, 0.0)})
        self.sim.peer_vel[1] = self.lua.table(0.1, -0.05)
        self.place(1.0, 3.2, 1.25)
        self.run_until(self.sim.now + 200)
        vn, ve, _ = self.command()
        self.assertAlmostEqual(vn, 0.1, delta=Q)
        self.assertAlmostEqual(ve, -0.05, delta=Q)
        # behind the slot by 0.1 m north: the correction adds kp * 0.1 northward
        self.place(0.9, 3.2, 1.25)
        self.run_until(self.sim.now + 200)
        vn, ve, _ = self.command()
        self.assertAlmostEqual(vn, 0.1 + 0.08, delta=Q)

    def test_follower_holds_then_lands_without_the_leader(self):
        self.take_off_and_fly(index=2)
        self.place(0.0, 0.6, 1.25)
        self.peers_at({2: (0.0, 0.0)})            # leader gone, vehicle 2 still heard
        self.run_until(self.sim.now + 1000)
        vn, ve, _ = self.command()
        self.assertAlmostEqual(vn, 0, places=3)
        self.assertAlmostEqual(ve, 0, places=3)
        self.assertEqual(self.state(), 4)
        self.run_until(self.sim.now + 5000)
        self.assertEqual(self.state(), 5)          # LAND
        self.assertIn("swarm: landing: leader lost", self.texts())


if __name__ == "__main__":
    unittest.main()
