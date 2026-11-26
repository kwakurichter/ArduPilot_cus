--[[
   swarm_hover.lua: synchronized swarm hover for the Crazyflie swarm simulation.

   Every vehicle takes off at the same instant, holds its pad position in its own
   altitude lane and lands at the same instant. The synchronized start, the climb,
   the separation guard and the landing are in modules/swarm_common.lua; this
   script only says where to be.
--]]

local swarm_common = require("swarm_common")

return swarm_common.run({
   name = "hover",
   reference = function(ctx, now)
      return ctx.site.pad_n, ctx.site.pad_e, 0, 0
   end,
})
