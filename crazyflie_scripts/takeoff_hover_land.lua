--[[
   Auto takeoff -> hover -> land test for the Crazyflie 2.1 Brushless.

   Sequence, run once per boot:
     1. wait for the operator to arm the vehicle
     2. switch to GUIDED and command a takeoff to TKO_ALT metres
     3. watch local NED z until the target height is reached
     4. hold position for TKO_HOVER_T seconds
     5. switch to LAND and wait for the automatic disarm

   Arming is the deliberate "go" signal, so nothing spins up on boot. See the
   note at the bottom of this file for making it fully self-arming instead.

   Parameters (all created by this script):
     TKO_ALT      target height above the EKF origin, metres
     TKO_HOVER_T  hover duration once the target height is reached, seconds
     TKO_TIMEOUT  give up and land if the climb takes longer than this, seconds
     TKO_TOL      how close to TKO_ALT counts as "arrived", metres
--]]

local MODE_GUIDED = 4
local MODE_LAND   = 9

local RUN_INTERVAL_MS = 100  -- 10 Hz is plenty for a state machine this simple

-- MAV_SEVERITY levels
local SEV_WARN = 4
local SEV_INFO = 6

-- Parameter table key must not collide with any other script on this board
local PARAM_TABLE_KEY = 82
assert(param:add_table(PARAM_TABLE_KEY, "TKO_", 4), "takeoff: could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "ALT",      1.0), "takeoff: could not add TKO_ALT")
assert(param:add_param(PARAM_TABLE_KEY, 2, "HOVER_T",  5.0), "takeoff: could not add TKO_HOVER_T")
assert(param:add_param(PARAM_TABLE_KEY, 3, "TIMEOUT", 15.0), "takeoff: could not add TKO_TIMEOUT")
assert(param:add_param(PARAM_TABLE_KEY, 4, "TOL",     0.10), "takeoff: could not add TKO_TOL")

local target_alt    = Parameter("TKO_ALT")
local hover_time    = Parameter("TKO_HOVER_T")
local climb_timeout = Parameter("TKO_TIMEOUT")
local alt_tol       = Parameter("TKO_TOL")

local STATE_WAIT_ARM = 0
local STATE_TAKEOFF  = 1
local STATE_HOVER    = 2
local STATE_LAND     = 3
local STATE_DONE     = 4

local state = STATE_WAIT_ARM
local state_start_ms = millis()

-- Height above the EKF origin. NED z is positive down, so negate it.
-- Returns nil when the EKF has no position estimate yet.
local function height_above_origin()
   local pos = ahrs:get_relative_position_NED_origin()
   if not pos then
      return nil
   end
   return -pos:z()
end

local function elapsed_s(now)
   return (now - state_start_ms):tofloat() / 1000.0
end

local function enter_state(new_state, now)
   state = new_state
   state_start_ms = now
end

-- Give up on the sequence and put it on the ground.
local function land_now(now, reason)
   gcs:send_text(SEV_WARN, "takeoff: " .. reason .. ", landing")
   vehicle:set_mode(MODE_LAND)
   enter_state(STATE_LAND, now)
end

-- Operator or a failsafe took the vehicle away from us; stop touching it.
local function stand_down(reason)
   gcs:send_text(SEV_WARN, "takeoff: " .. reason .. ", script standing down")
   state = STATE_DONE
end

local function update_wait_arm(now)
   if not arming:is_armed() then
      return
   end

   -- GUIDED needs a position estimate. On this airframe that means the flow
   -- deck and rangefinder have to be feeding EKF3 before we leave the ground.
   if height_above_origin() == nil then
      gcs:send_text(SEV_WARN, "takeoff: no EKF position estimate, disarming")
      arming:disarm()
      return
   end

   if not vehicle:set_mode(MODE_GUIDED) then
      gcs:send_text(SEV_WARN, "takeoff: GUIDED refused, disarming")
      arming:disarm()
      return
   end

   if not vehicle:start_takeoff(target_alt:get()) then
      gcs:send_text(SEV_WARN, "takeoff: start_takeoff rejected, disarming")
      arming:disarm()
      return
   end

   gcs:send_text(SEV_INFO, string.format("takeoff: climbing to %.2f m", target_alt:get()))
   enter_state(STATE_TAKEOFF, now)
end

local function update_takeoff(now)
   if not arming:is_armed() then
      stand_down("disarmed during climb")
      return
   end
   if vehicle:get_mode() ~= MODE_GUIDED then
      stand_down("left GUIDED during climb")
      return
   end

   local height = height_above_origin()
   if height and height >= (target_alt:get() - alt_tol:get()) then
      gcs:send_text(SEV_INFO, string.format("takeoff: reached %.2f m, hovering %.1f s",
                                            height, hover_time:get()))
      enter_state(STATE_HOVER, now)
   elseif elapsed_s(now) > climb_timeout:get() then
      land_now(now, "climb timed out")
   end
end

local function update_hover(now)
   if not arming:is_armed() then
      stand_down("disarmed during hover")
      return
   end
   if vehicle:get_mode() ~= MODE_GUIDED then
      stand_down("left GUIDED during hover")
      return
   end

   if elapsed_s(now) > hover_time:get() then
      gcs:send_text(SEV_INFO, "takeoff: hover complete, landing")
      vehicle:set_mode(MODE_LAND)
      enter_state(STATE_LAND, now)
   end
end

local function update_land()
   -- Copter disarms itself once the landing detector is satisfied
   if not arming:is_armed() then
      gcs:send_text(SEV_INFO, "takeoff: landed and disarmed, sequence complete")
      state = STATE_DONE
   end
end

function update()
   local now = millis()

   if state == STATE_WAIT_ARM then
      if ahrs:initialised() then
         update_wait_arm(now)
      end
   elseif state == STATE_TAKEOFF then
      update_takeoff(now)
   elseif state == STATE_HOVER then
      update_hover(now)
   elseif state == STATE_LAND then
      update_land()
   end

   return update, RUN_INTERVAL_MS
end

gcs:send_text(SEV_INFO, "takeoff: loaded, waiting for arm")

return update, RUN_INTERVAL_MS

--[[
   To make this self-arming instead of waiting for the operator, replace the
   `if not arming:is_armed() then return end` guard at the top of
   update_wait_arm() with a call to arming:arm(). Do that only once you have
   watched the sequence work end to end with a manual arm.
--]]
