--[[
   Auto takeoff -> square -> land test for the Crazyflie 2.1 Brushless.

   Sequence, run once per boot:
     1. wait for the operator to arm the vehicle
     2. switch to GUIDED and command a takeoff to SQR_ALT metres
     3. record the position reached as the first corner of the square
     4. fly the four legs below, settling SQR_SETTLE seconds at each corner
     5. switch to LAND and wait for the automatic disarm

   The square is traced in the EKF origin's NED frame at constant altitude,
   starting and finishing at the same point:

           corner 3 <-------- corner 2
              |                  ^
              |                  |          N (x)
              v                  |          ^
           corner 4 --------> corner 1      +--> E (y)
          (= start)

   Legs are commanded in the NED frame rather than the body frame on purpose.
   With no compass the EKF's yaw reference is arbitrary but self consistent,
   so NED offsets trace a repeatable square while "left" would drift with yaw.

   Parameters (all created by this script):
     SQR_ALT      takeoff height above the EKF origin, metres
     SQR_SIZE     length of each leg, metres
     SQR_SETTLE   hold time at each corner, seconds
     SQR_TOL      arrival radius for a corner, metres
     SQR_TIMEOUT  give up and land if a leg takes longer than this, seconds
     SQR_CLIMB_T  give up and land if the climb takes longer than this, seconds
--]]

local MODE_GUIDED = 4
local MODE_LAND   = 9

local RUN_INTERVAL_MS = 100  -- 10 Hz

-- MAV_SEVERITY levels
local SEV_WARN = 4
local SEV_INFO = 6

-- Parameter table key must not collide with any other script on this board
local PARAM_TABLE_KEY = 83
assert(param:add_table(PARAM_TABLE_KEY, "SQR_", 6), "square: could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "ALT",      1.0), "square: could not add SQR_ALT")
assert(param:add_param(PARAM_TABLE_KEY, 2, "SIZE",     1.0), "square: could not add SQR_SIZE")
assert(param:add_param(PARAM_TABLE_KEY, 3, "SETTLE",   5.0), "square: could not add SQR_SETTLE")
assert(param:add_param(PARAM_TABLE_KEY, 4, "TOL",      0.15), "square: could not add SQR_TOL")
assert(param:add_param(PARAM_TABLE_KEY, 5, "TIMEOUT", 10.0), "square: could not add SQR_TIMEOUT")
assert(param:add_param(PARAM_TABLE_KEY, 6, "CLIMB_T", 15.0), "square: could not add SQR_CLIMB_T")

local target_alt    = Parameter("SQR_ALT")
local square_size   = Parameter("SQR_SIZE")
local settle_time   = Parameter("SQR_SETTLE")
local arrive_tol    = Parameter("SQR_TOL")
local leg_timeout   = Parameter("SQR_TIMEOUT")
local climb_timeout = Parameter("SQR_CLIMB_T")

-- Corner offsets from the start point, in units of SQR_SIZE, as {north, east}
local leg_offsets = {
   {1, 0},
   {1, 1},
   {0, 1},
   {0, 0},
}

local STATE_WAIT_ARM = 0
local STATE_TAKEOFF  = 1
local STATE_GOTO     = 2
local STATE_SETTLE   = 3
local STATE_LAND     = 4
local STATE_DONE     = 5

local state = STATE_WAIT_ARM
local state_start_ms = millis()

local square_origin = nil  -- Vector3f, position at the top of the climb
local current_target = nil -- Vector3f, corner currently being flown to
local corner_index = 0

-- Position relative to the EKF origin in NED metres, or nil if the EKF has
-- no estimate yet.
local function position_NED()
   return ahrs:get_relative_position_NED_origin()
end

-- Height above the EKF origin. NED z is positive down, so negate it.
local function height_above_origin()
   local pos = position_NED()
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
   gcs:send_text(SEV_WARN, "square: " .. reason .. ", landing")
   vehicle:set_mode(MODE_LAND)
   enter_state(STATE_LAND, now)
end

-- Operator or a failsafe took the vehicle away from us; stop touching it.
local function stand_down(reason)
   gcs:send_text(SEV_WARN, "square: " .. reason .. ", script standing down")
   state = STATE_DONE
end

-- True while we still hold the vehicle. Anything else means hands off.
local function still_ours()
   return arming:is_armed() and vehicle:get_mode() == MODE_GUIDED
end

local function corner_target(index)
   local offset = leg_offsets[index]
   local size = square_size:get()
   local target = Vector3f()
   target:x(square_origin:x() + offset[1] * size)
   target:y(square_origin:y() + offset[2] * size)
   target:z(-target_alt:get())
   return target
end

-- Command the next corner and move into the GOTO state.
local function command_corner(index, now)
   local target = corner_target(index)
   if not vehicle:set_target_pos_NED(target, false, 0, false, 0, false, false) then
      land_now(now, string.format("corner %d target rejected", index))
      return
   end
   current_target = target
   corner_index = index
   gcs:send_text(SEV_INFO, string.format("square: leg %d of %d, to N%.2f E%.2f",
                                         index, #leg_offsets, target:x(), target:y()))
   enter_state(STATE_GOTO, now)
end

local function update_wait_arm(now)
   if not arming:is_armed() then
      return
   end

   -- GUIDED needs a position estimate. On this airframe that means the flow
   -- deck and rangefinder have to be feeding EKF3 before we leave the ground.
   if position_NED() == nil then
      gcs:send_text(SEV_WARN, "square: no EKF position estimate, disarming")
      arming:disarm()
      return
   end

   if not vehicle:set_mode(MODE_GUIDED) then
      gcs:send_text(SEV_WARN, "square: GUIDED refused, disarming")
      arming:disarm()
      return
   end

   if not vehicle:start_takeoff(target_alt:get()) then
      gcs:send_text(SEV_WARN, "square: start_takeoff rejected, disarming")
      arming:disarm()
      return
   end

   gcs:send_text(SEV_INFO, string.format("square: climbing to %.2f m", target_alt:get()))
   enter_state(STATE_TAKEOFF, now)
end

local function update_takeoff(now)
   if not still_ours() then
      stand_down("interrupted during climb")
      return
   end

   local height = height_above_origin()
   if height and height >= (target_alt:get() - arrive_tol:get()) then
      -- Freeze the corner the square is built from, so drift during the climb
      -- doesn't skew the pattern.
      square_origin = position_NED()
      if square_origin == nil then
         land_now(now, "lost position estimate at top of climb")
         return
      end
      gcs:send_text(SEV_INFO, string.format("square: at %.2f m, starting %.2f m square",
                                            height, square_size:get()))
      command_corner(1, now)
   elseif elapsed_s(now) > climb_timeout:get() then
      land_now(now, "climb timed out")
   end
end

local function update_goto(now)
   if not still_ours() then
      stand_down("interrupted in transit")
      return
   end

   local pos = position_NED()
   if pos == nil then
      land_now(now, "lost position estimate in transit")
      return
   end

   local north_err = pos:x() - current_target:x()
   local east_err = pos:y() - current_target:y()
   local distance = math.sqrt(north_err * north_err + east_err * east_err)

   if distance <= arrive_tol:get() then
      gcs:send_text(SEV_INFO, string.format("square: corner %d reached, settling %.1f s",
                                            corner_index, settle_time:get()))
      enter_state(STATE_SETTLE, now)
   elseif elapsed_s(now) > leg_timeout:get() then
      land_now(now, string.format("leg %d timed out %.2f m short", corner_index, distance))
   end
end

local function update_settle(now)
   if not still_ours() then
      stand_down("interrupted while settling")
      return
   end

   if elapsed_s(now) < settle_time:get() then
      return
   end

   if corner_index >= #leg_offsets then
      gcs:send_text(SEV_INFO, "square: complete, landing")
      vehicle:set_mode(MODE_LAND)
      enter_state(STATE_LAND, now)
   else
      command_corner(corner_index + 1, now)
   end
end

local function update_land()
   -- Copter disarms itself once the landing detector is satisfied
   if not arming:is_armed() then
      gcs:send_text(SEV_INFO, "square: landed and disarmed, sequence complete")
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
   elseif state == STATE_GOTO then
      update_goto(now)
   elseif state == STATE_SETTLE then
      update_settle(now)
   elseif state == STATE_LAND then
      update_land()
   end

   return update, RUN_INTERVAL_MS
end

gcs:send_text(SEV_INFO, "square: loaded, waiting for arm")

return update, RUN_INTERVAL_MS
