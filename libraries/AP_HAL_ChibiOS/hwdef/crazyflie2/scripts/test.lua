-- test.lua

local TAKEOFF_ALT = 1      -- metres
local FORWARD_DIST = 3     -- metres

local state = 0           -- 0=waiting for arm, 1=mode set, 2=takeoff cmd, 3=waiting climb, 4=goto, 5=rtl

function update()
    -- 1) wait for arm
    if state == 0 then
        if not arming:is_armed() then
            gcs:send_text(4,"Waiting for ARM…")
            return update,100
        end
        state = 1
    end

    -- 2) Set EKF origin
    if state == 1 then
        local origin_ok, origin_err = ahrs:get_origin()
        if origin_ok then
            gcs:send_text(3, "EKF origin already set — aborting")
            return nill
        end

        local loc = Location()
        loc:lat(-353632640); loc:lng(1491652352); loc:alt(58409)
        if not ahrs:set_origin(loc) then
            gcs:send_text(3, "Failed to set EKF origin")
            return nil
        end
        gcs:send_text(6, string.format("Origin Set: %.7f, %.7f, %.1f", loc:lat()/1e7, loc:lng()/1e7, loc:alt()/100))
        
        state = 2
    end

    -- 3) switch to GUIDED
    if state == 2 then
        if vehicle:set_mode(4) then       -- GUIDED=4
            gcs:send_text(6,"Mode=GUIDED")
            state = 3
        else
            gcs:send_text(3,"GUIDED failed")
            return nil
        end
    end

    -- 4) pause and poll
    if state == 3 then
        local ned
        repeat
            ned = ahrs:get_relative_position_NED_home()
            if not ned then
                gcs:send_text(4, "Waiting for EKF local position…")
                return update, 100   -- wait 100 ms, then check again
            end
        until ned

        gcs:send_text(6, "Local position OK — now taking off")
        state = 4
    end

    -- 5) send takeoff once
    if state == 4 then
        if vehicle:start_takeoff(TAKEOFF_ALT) then
            gcs:send_text(6,string.format("Takeoff to %.1fm",TAKEOFF_ALT))
            state = 5
        else
            gcs:send_text(3,"Takeoff failed")
            return nil
        end
    end

    -- 6) wait until we’re up
    if state == 5 then
        local ned = ahrs:get_relative_position_NED_home()
        if not ned or (-ned:z() < TAKEOFF_ALT*0.9) then
            return update,200     -- still climbing
        end
        gcs:send_text(6,"Climb complete")
        state = 6
    end

    -- 7) move forward
    if state == 6 then
        local pos = ahrs:get_position()
        if pos then
            pos:offset(0, FORWARD_DIST)   -- north=0, east=+3 m → forward
            vehicle:set_target_location(pos)
            gcs:send_text(6,string.format("Goto +%dm",FORWARD_DIST))
            state = 7
        else
            gcs:send_text(3,"No position")
            return nil
        end
    end

    -- 8) RTL
    if state == 7 then
        if vehicle:set_mode(6) then    -- RTL=6
            gcs:send_text(6,"Mode=RTL")
        else
            gcs:send_text(3,"RTL failed")
        end
        return nil  -- mission done
    end
end

-- kick it off 5s after load
return update,5000
