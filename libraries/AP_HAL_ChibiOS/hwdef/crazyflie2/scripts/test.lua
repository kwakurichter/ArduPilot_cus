-- Test for Autonomous flight

function update () -- periodic function that will be called
    gcs:send_text(6, "Autonomous test flight START...")

    -- 1. Wait until drone is Armed
    if not vehicle:armed() then
        gcs:send_text(4, "Waiting for ARM...")
        return update, 5000 -- retry in 5s
    end
    gcs:send_text(6, "Armed! Starting mission...")

    -- 2. Wait for EKF to init
    if not ahrs:initialised() then
        gcs:send_text(4, "Waiting for EKF init...")
        return update, 5000 -- retry in 5s
    end

    -- 3. Set EKF origin (only once)
    local origin_ok, origin_err = ahrs:get_origin()
    if origin_ok then
        gcs:send_text(3, "EKF origin already set — aborting")
        return nil
    end

    local loc = Location()
    loc:lat(-353632640); loc:lng(1491652352); loc:alt(58409)
    if not ahrs:set_origin(loc) then
        gcs:send_text(3, "Failed to set EKF origin")
        return nil
    end
    gcs:send_text(6, string.format("Origin Set: %.7f, %.7f, %.1f", loc:lat()/1e7, loc:lng()/1e7, loc:alt()/100))

    -- 4. Switch to GUIDED mode
    local GUIDED = 4
    if not vehicle:set_mode(GUIDED) then
        gcs:send_text(3, "Failed to switch to GUIDED mode")
        return nil
    end
    gcs:send_text(6, "Switched to GUIDED mode")

    -- 5. Climb to altitude and then move
    if ahrs:get_altitude() < 0.9 then
        gcs:send_text(4, "Climbing...")
        return update, 1000
    end
    local pos = ahrs:get_position() -- get current Location
    if not pos then
        gcs:send_text(3, "No position — aborting")
        return nil
    end
    pos:offset(3, 0)    -- translate 3m North, 0m East
    vehicle:set_target_location(pos)

    -- 6. Once done, switch to RTL (Return-to-Launch) mode
    local RTL_MODE = 6
    if not vehicle:set_mode(RTL_MODE) then
        gcs:send_text(3, "Failed to switch to RTL mode")
    end
    gcs:send_text(6, "Switched to RTL mode. Mission complete.")
  
    -- return update, 1000 -- request "update" to be rerun again 1000 milliseconds from now
    return nil -- don't re-run
end
  
return update, 5000   -- request "update" to be the first time 5000 milliseconds after script is loaded