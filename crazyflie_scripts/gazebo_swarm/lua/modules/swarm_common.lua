--[[
   swarm_common.lua: everything a swarm mission script shares in the Crazyflie swarm
   simulation, so a mission only has to say where the vehicle should be.

   A mission script is

      local swarm_common = require("swarm_common")
      return swarm_common.run({
         name = "hover",
         reference = function(ctx, now)
            -- where this vehicle should be now (world north/east, m) and how fast
            -- that point moves (m/s); return nil to hold position
            return ctx.site.pad_n, ctx.site.pad_e, 0, 0
         end,
      })

   run() owns the rest of the flight:
     SITE      indoor profile only: set the EKF origin to this vehicle's pad in the
               shared world frame, so every vehicle's position (and what AP_SwarmMesh
               broadcasts) is in one frame
     WAIT      wait for a position, every peer on the mesh, GUIDED (selected while
               disarmed; Copter refuses it without a usable position) and the
               pre-arm checks
     READY     all of that held for READY_HOLD_MS; wait for CFSW_START
     TAKEOFF   at CFSW_START (this vehicle's own millis()): GUIDED, arm, take off to
               this vehicle's altitude lane
     FLY       track the mission's reference in the world frame, in the altitude
               lane, with a simple separation guard against peers, until the common
               end time; the mission's own clock starts at ctx.t0 for every vehicle
     LAND      LAND until disarmed
     DONE / ABORT

   CFSW_START is a time in this vehicle's own boot clock (ms). The orchestrator
   (run_swarm.py) measures each vehicle's clock with MAVLink TIMESYNC and sets
   CFSW_START on every vehicle so they all start at the same instant. Then
   ctx.t0 = CFSW_START + climb_s and the landing is at ctx.t0 + duration, the same
   instants on every vehicle.

   The context passed to reference():
     ctx.site       this vehicle's swarm_site.lua (index, sysid, pads, timing, ...)
     ctx.t0         common mission start, ms in this vehicle's clock
     ctx.peers      sysid -> {n, e, vn, ve} for every peer AP_SwarmMesh has a fresh
                    position for (world frame; vn/ve nil without a fresh velocity)
     ctx.land(now, why)  end the mission early

   Status for the orchestrator (NAMED_VALUE_FLOAT): SWSTATE (state number above),
   SWPEERS (peers with a fresh position), SWTKO (millis() at arming) and, in FLY,
   SWERR (horizontal distance to the reference, m).

   Parameters (created here):
     CFSW_START   synchronized start time, ms since boot (0 = not set)
     CFSW_SPEED   maximum horizontal speed, m/s
     CFSW_KP      horizontal position gain, 1/s
     CFSW_AVOID   separation guard radius, m
     CFSW_PUSH    maximum guard velocity, m/s
     CFSW_TOL     altitude tolerance for the end of the climb, m
     CFSW_VZMAX   maximum vertical speed in FLY, m/s
--]]

local M = {}

local site = require("swarm_site")

local MODE_GUIDED = 4
local MODE_LAND = 9
local TICK_MS = 100
local REPORT_MS = 500
local READY_HOLD_MS = 2000
local READY_DROP_MS = 1000     -- READY is lost only after this long not ready
local START_GRACE_MS = 1000    -- keep trying to take off this long after the start
local START_RETRY_MS = 20
local WAIT_REPORT_MS = 3000    -- rate limit for the waiting reason
local LATE_START_MS = 500      -- a start this far in the past is refused
local PEERS_HOLD_MS = 3000     -- no peer heard this long: hold position
local PEERS_LAND_MS = 10000    -- no peer heard this long: land
local GUARD_FILTER = 0.35
local KZ = 1.0                 -- vertical position gain, 1/s

local SEV_ERROR = 3
local SEV_WARN = 4
local SEV_INFO = 6

local S_SITE, S_WAIT, S_READY, S_TAKEOFF, S_FLY, S_LAND, S_DONE, S_ABORT = 0, 1, 2, 3, 4, 5, 6, 7
local STATE_NAMES = {[0] = "SITE", "WAIT", "READY", "TAKEOFF", "FLY", "LAND", "DONE", "ABORT"}

-- key 82 = TKO_, 83 = SQR_ (crazyflie_scripts); 85 is free
local PARAM_TABLE_KEY = 85
assert(param:add_table(PARAM_TABLE_KEY, "CFSW_", 7), "swarm: could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "START", 0), "swarm: CFSW_START")
assert(param:add_param(PARAM_TABLE_KEY, 2, "SPEED", 0.25), "swarm: CFSW_SPEED")
assert(param:add_param(PARAM_TABLE_KEY, 3, "KP", 0.8), "swarm: CFSW_KP")
assert(param:add_param(PARAM_TABLE_KEY, 4, "AVOID", 0.45), "swarm: CFSW_AVOID")
assert(param:add_param(PARAM_TABLE_KEY, 5, "PUSH", 0.6), "swarm: CFSW_PUSH")
assert(param:add_param(PARAM_TABLE_KEY, 6, "TOL", 0.12), "swarm: CFSW_TOL")
assert(param:add_param(PARAM_TABLE_KEY, 7, "VZMAX", 0.3), "swarm: CFSW_VZMAX")

local p_start = Parameter("CFSW_START")
local p_speed = Parameter("CFSW_SPEED")
local p_kp = Parameter("CFSW_KP")
local p_avoid = Parameter("CFSW_AVOID")
local p_push = Parameter("CFSW_PUSH")
local p_tol = Parameter("CFSW_TOL")
local p_vzmax = Parameter("CFSW_VZMAX")

-- ------------------------------------------------------------------ geometry

local function lane_altitude(index, count)
   local lanes = math.max(1, math.min(site.lane_count, count))
   local lane = index % lanes
   return site.alt + (lane - (lanes - 1) / 2) * site.lane_spacing
end

local function world_origin()
   local origin = Location()
   origin:lat(site.origin_lat_e7)
   origin:lng(site.origin_lng_e7)
   origin:alt(site.origin_alt_cm)
   return origin
end

local ORIGIN = world_origin()
local LANE_ALT = lane_altitude(site.index, site.count)
local CLIMB_MS = math.floor(site.climb_s * 1000)
local FLY_MS = math.floor(site.duration * 1000)

-- world north/east (m) of a location
local function world_ne(loc)
   local d = ORIGIN:get_distance_NE(loc)
   return d:x(), d:y()
end

local function saturate(x, y, limit)
   local n = math.sqrt(x * x + y * y)
   if n > limit and n > 0 then
      return x * limit / n, y * limit / n
   end
   return x, y
end

-- ------------------------------------------------------------------ state

local mission = nil
local ctx = {site = site, peers = {}, t0 = nil}
local state = S_SITE
local ready_since = nil
local not_ready_since = nil
local start_ms = nil
local start_seen_ms = nil
local tko_ms = nil
local land_sent = false
local last_report = 0
local last_origin_try = 0
local last_mode_try = 0
local last_wait_reason = nil
local last_wait_report = -100000
local last_peer_ms = nil
local guard_n, guard_e = 0, 0
local fresh_peers = 0
local tracking_error = nil

local function enter(new_state, why)
   state = new_state
   local text = string.format("swarm: %s", STATE_NAMES[new_state])
   if why then
      text = text .. " (" .. why .. ")"
   end
   local severity = SEV_INFO
   if new_state == S_ABORT then
      severity = SEV_ERROR
   end
   gcs:send_text(severity, text)
end

local function abort(why)
   if arming:is_armed() then
      vehicle:set_mode(MODE_LAND)
   end
   enter(S_ABORT, why)
end

-- ctx.land(): a mission ends its flight early
function ctx.land(_, why)
   if state == S_TAKEOFF or state == S_FLY then
      gcs:send_text(SEV_WARN, "swarm: landing: " .. tostring(why))
      enter(S_LAND, why)
   end
end

local function update_peers(now)
   fresh_peers = 0
   for _, sysid in ipairs(site.sysids) do
      if sysid ~= site.sysid then
         local loc = swarm:get_peer_location(sysid)
         if loc then
            local n, e = world_ne(loc)
            local peer = {n = n, e = e}
            local vel = swarm:get_peer_velocity_NED(sysid)
            if vel then
               peer.vn, peer.ve = vel:x(), vel:y()
            end
            ctx.peers[sysid] = peer
            fresh_peers = fresh_peers + 1
         else
            ctx.peers[sysid] = nil
         end
      end
   end
   if fresh_peers > 0 or site.count <= 1 then
      last_peer_ms = now
   end
end

local function report(now)
   if now - last_report < REPORT_MS then
      return
   end
   last_report = now
   gcs:send_named_float("SWSTATE", state)
   gcs:send_named_float("SWPEERS", fresh_peers)
   if tko_ms then
      gcs:send_named_float("SWTKO", tko_ms)
   end
   if state == S_FLY and tracking_error then
      gcs:send_named_float("SWERR", tracking_error)
   end
end

local function altitude()
   local d = ahrs:get_relative_position_D_home()
   if d == nil then
      return nil
   end
   return -d
end

-- ------------------------------------------------------------------ on the ground

local function update_site(now)
   if site.profile ~= "indoor" or ahrs:get_origin() then
      enter(S_WAIT)
      return
   end
   if now - last_origin_try < 1000 or not ahrs:initialised() then
      return
   end
   last_origin_try = now
   local pad = world_origin()
   pad:offset(site.pad_n, site.pad_e)
   if ahrs:set_origin(pad) then
      gcs:send_text(SEV_INFO, string.format("swarm: origin set at pad N%.2f E%.2f", site.pad_n, site.pad_e))
   end
end

-- Ready means this vehicle could take off now. Copter accepts GUIDED only with a
-- usable position, the same test arming applies in GUIDED, so the mode is selected
-- (disarmed) as part of becoming ready.
local function swarm_ready(now)
   if not ahrs:healthy() then
      return false, "AHRS not healthy"
   end
   if ahrs:get_location() == nil then
      return false, "no position"
   end
   if fresh_peers < site.count - 1 then
      return false, string.format("%d of %d peers", fresh_peers, site.count - 1)
   end
   if vehicle:get_mode() ~= MODE_GUIDED then
      if now - last_mode_try >= 1000 then
         last_mode_try = now
         vehicle:set_mode(MODE_GUIDED)
      end
      return false, "GUIDED not accepted yet"
   end
   if not arming:pre_arm_checks() then
      return false, "pre-arm checks"
   end
   return true
end

local function start_time(now)
   local value = p_start:get()
   if value == nil or value <= 0 then
      return nil
   end
   local t = math.floor(value + 0.5)
   if start_ms ~= t then
      start_ms = t
      start_seen_ms = now
      ctx.t0 = t + CLIMB_MS
      gcs:send_text(SEV_INFO, string.format("swarm: start at %d ms (in %d ms)", t, t - now))
   end
   return t
end

local function try_takeoff(now)
   if not vehicle:set_mode(MODE_GUIDED) then
      return false, "GUIDED refused"
   end
   if not arming:arm() then
      return false, "arming refused"
   end
   tko_ms = now
   if not vehicle:start_takeoff(LANE_ALT) then
      vehicle:set_mode(MODE_LAND)
      return false, "takeoff refused"
   end
   return true
end

-- returns the delay to the next run
local function update_wait(now)
   local ready, reason = swarm_ready(now)
   if not ready and reason ~= last_wait_reason and now - last_wait_report >= WAIT_REPORT_MS then
      last_wait_reason = reason
      last_wait_report = now
      gcs:send_text(SEV_INFO, "swarm: waiting: " .. reason)
   end
   if ready then
      not_ready_since = nil
      if ready_since == nil then
         ready_since = now
      end
      if state == S_WAIT and now - ready_since >= READY_HOLD_MS then
         enter(S_READY)
      end
   else
      ready_since = nil
      if not_ready_since == nil then
         not_ready_since = now
      end
      -- checks such as the EKF's can flicker briefly; only a lasting loss counts
      if state == S_READY and now - not_ready_since >= READY_DROP_MS then
         enter(S_WAIT, "readiness lost: " .. reason)
      end
   end

   local t = start_time(now)
   if t == nil then
      return TICK_MS
   end
   if start_seen_ms > t + LATE_START_MS then
      -- the start reached us too late to be synchronized with the others
      abort(string.format("start time %d ms had passed when it arrived", t))
      return TICK_MS
   end
   if now < t then
      -- wake up exactly at the start time
      return math.max(1, math.min(TICK_MS, t - now))
   end
   if state ~= S_READY then
      abort("not ready at the start time (" .. tostring(reason) .. ")")
      return TICK_MS
   end
   local ok, why = false, reason
   if ready then
      ok, why = try_takeoff(now)
   end
   if ok then
      enter(S_TAKEOFF, string.format("%.2f m, late %d ms", LANE_ALT, now - start_ms))
      return TICK_MS
   end
   if now - t >= START_GRACE_MS then
      abort("could not take off: " .. tostring(why))
      return TICK_MS
   end
   return START_RETRY_MS
end

-- ------------------------------------------------------------------ in the air

local function separation_guard(n, e)
   local avoid = p_avoid:get()
   local push_n, push_e = 0, 0
   if avoid > 0 then
      local gain = p_push:get() / avoid
      for _, peer in pairs(ctx.peers) do
         local dn, de = n - peer.n, e - peer.e
         local d = math.sqrt(dn * dn + de * de)
         if d < avoid and d > 1e-3 then
            push_n = push_n + gain * (avoid - d) * dn / d
            push_e = push_e + gain * (avoid - d) * de / d
         end
      end
      push_n, push_e = saturate(push_n, push_e, p_push:get())
   end
   guard_n = guard_n + GUARD_FILTER * (push_n - guard_n)
   guard_e = guard_e + GUARD_FILTER * (push_e - guard_e)
   return guard_n, guard_e
end

local function command_fly(now)
   local loc = ahrs:get_location()
   local alt = altitude()
   if loc == nil or alt == nil then
      vehicle:set_target_velocity_NED(Vector3f())
      return
   end
   local n, e = world_ne(loc)

   -- track the mission's reference: its velocity plus a correction toward it
   local vn, ve = 0, 0
   tracking_error = nil
   local lost = last_peer_ms ~= nil and now - last_peer_ms > PEERS_HOLD_MS
   local rn, re, rvn, rve = mission.reference(ctx, now)
   if rn ~= nil and not lost then
      local kp = p_kp:get()
      vn, ve = saturate((rvn or 0) + kp * (rn - n), (rve or 0) + kp * (re - e), p_speed:get())
      tracking_error = math.sqrt((rn - n) ^ 2 + (re - e) ^ 2)
   end
   local gn, ge = separation_guard(n, e)
   vn, ve = vn + gn, ve + ge

   local vzmax = p_vzmax:get()
   local vup = math.max(-vzmax, math.min(vzmax, KZ * (LANE_ALT - alt)))
   local velocity = Vector3f()
   velocity:x(vn)
   velocity:y(ve)
   velocity:z(-vup)
   vehicle:set_target_velocity_NED(velocity)
end

local function update_flight(now)
   if not arming:is_armed() then
      if state == S_LAND then
         enter(S_DONE)
      else
         abort("disarmed in flight")
      end
      return
   end
   if state ~= S_LAND then
      if last_peer_ms ~= nil and now - last_peer_ms > PEERS_LAND_MS then
         ctx.land(now, "peers lost")
      elseif now >= ctx.t0 + FLY_MS then
         enter(S_LAND)
      end
   end
   if state == S_TAKEOFF then
      local alt = altitude()
      if alt ~= nil and alt >= LANE_ALT - p_tol:get() then
         enter(S_FLY, string.format("%.2f m", alt))
      elseif now > ctx.t0 then
         ctx.land(now, "climb timed out")
      end
   elseif state == S_FLY then
      command_fly(now)
   end
   if state == S_LAND and not land_sent then
      land_sent = vehicle:set_mode(MODE_LAND)
   end
end

-- ------------------------------------------------------------------ main loop

local function validate_site()
   assert(site.index ~= nil and site.count ~= nil and site.sysid ~= nil, "swarm_site: identity")
   assert(site.origin_lat_e7 and site.origin_lng_e7 and site.origin_alt_cm, "swarm_site: origin")
   assert(site.pad_n and site.pad_e and site.pads, "swarm_site: pads")
   assert(site.alt and site.lane_count and site.lane_spacing, "swarm_site: altitudes")
   assert(site.climb_s and site.duration, "swarm_site: timing")
end

local function update()
   local now = millis():toint()
   update_peers(now)
   local delay = TICK_MS
   if state == S_SITE then
      update_site(now)
   elseif state == S_WAIT or state == S_READY then
      delay = update_wait(now)
   elseif state == S_TAKEOFF or state == S_FLY or state == S_LAND then
      update_flight(now)
   end
   report(now)
   if state == S_DONE or state == S_ABORT then
      -- keep reporting so the orchestrator sees the final state
      return update, REPORT_MS
   end
   return update, delay
end

local function protected_update()
   local ok, result, delay = pcall(update)
   if not ok then
      gcs:send_text(SEV_ERROR, "swarm: error " .. tostring(result))
      if arming:is_armed() then
         vehicle:set_mode(MODE_LAND)
      end
      gcs:send_named_float("SWSTATE", S_ABORT)
      return
   end
   return protected_update, delay
end

-- Start a mission: {name = string, reference = function(ctx, now)}.
-- Returns the function and delay a script returns to the scheduler.
function M.run(m)
   assert(m and m.name and m.reference, "swarm_common.run: mission needs a name and a reference()")
   mission = m
   validate_site()
   assert(site.mission == m.name, string.format("swarm_site is for mission %s, not %s",
      tostring(site.mission), m.name))
   -- STATUSTEXT holds 50 characters
   gcs:send_text(SEV_INFO, string.format("swarm: %s %d/%d, sysid %d, lane %.2f m",
      m.name, site.index + 1, site.count, site.sysid, LANE_ALT))
   return protected_update, 1000
end

return M
