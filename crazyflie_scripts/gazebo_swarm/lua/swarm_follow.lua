--[[
   swarm_follow.lua: leader-follower formation over AP_SwarmMesh.

   The leader (site.leader_sysid) flies a circle. Every other vehicle keeps the
   offset from the leader that it had on the ground, so the formation keeps its
   shape as it moves around the circle.

   A follower knows nothing about the leader except what AP_SwarmMesh tells it:
   swarm:get_peer_location() and swarm:get_peer_velocity_NED() for the leader's
   sysid, collected each tick into ctx.peers by swarm_common. The follower aims at
   the leader's position plus its offset, and moves with the leader's velocity
   (feed-forward), so it does not trail behind.

   The synchronized start, the climb into altitude lanes, the separation guard and
   the landing are in modules/swarm_common.lua.
--]]

local swarm_common = require("swarm_common")
local site = require("swarm_site")

local LEADER = site.leader_sysid
local LEADER_LOST_MS = 5000   -- a follower holds while the leader is missing, then lands

-- this vehicle's place in the formation: its pad relative to the leader's pad
local leader_pad = site.pads[LEADER]
local SLOT_N = site.pad_n - leader_pad.n
local SLOT_E = site.pad_e - leader_pad.e

-- the leader's circle starts at its pad, with the centre circle_radius south of it
local RADIUS = site.circle_radius
local CENTRE_N = leader_pad.n - RADIUS
local CENTRE_E = leader_pad.e
local RATE = site.circle_speed / RADIUS   -- rad/s, clockwise seen from above

local function leader_reference(ctx, now)
   local t = math.max(0, now - ctx.t0) / 1000   -- every vehicle starts its clock at ctx.t0
   local angle = RATE * t
   local n = CENTRE_N + RADIUS * math.cos(angle)
   local e = CENTRE_E + RADIUS * math.sin(angle)
   if now < ctx.t0 then
      return n, e, 0, 0                          -- wait over the pad for the others
   end
   local speed = site.circle_speed
   return n, e, -speed * math.sin(angle), speed * math.cos(angle)
end

local leader_seen_ms = nil

local function follower_reference(ctx, now)
   local leader = ctx.peers[LEADER]
   if leader == nil then
      if leader_seen_ms and now - leader_seen_ms > LEADER_LOST_MS then
         ctx.land(now, "leader lost")
      end
      return nil                                 -- hold position
   end
   leader_seen_ms = now
   return leader.n + SLOT_N, leader.e + SLOT_E, leader.vn, leader.ve
end

local reference = follower_reference
if site.sysid == LEADER then
   reference = leader_reference
end

return swarm_common.run({name = "follow", reference = reference})
