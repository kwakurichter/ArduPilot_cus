# Lua Scripting Guide
## What is Lua Scripting?
ArduPilot Copter’s Lua scripting provides a lightweight, isolated way to add or tweak behaviour without touching the core C++ flight code. Once you enable scripting by setting SCR_ENABLE to 1 and place your .lua files in the APM/scripts folder on the autopilot’s SD card (or embed them in firmware), they’re loaded at power-up and run in parallel with the main flight loop.

Each script runs in its own environment with a fixed time slice, can register callback functions (for example to run every 1 s), and has access to a API for reading vehicle state (GPS, attitude, battery, RC channels, etc.), manipulating outputs (servos, relays, LEDs), issuing MAVLink commands, etc.

Because scripts execute at low priority in a VM, even long-running or misbehaving scripts won’t block or crash the autopilot. This makes Lua ideal for rapid prototyping or mission-specific customizations. Everything from simple applets (e.g. triggering a floodlight on an RC switch) to full driver support for unsupported peripherals, without the cycle of rebuilding and reflashing the firmware (although currently reflashing is necessary on Crazyflie until external SD card support is enabled).

You can also expose script-generated parameters and assign RC channels as script inputs, giving you powerful, field-configurable control logic that is isolated from the core flight stack

## Lua on Crazyflie
To get Lua scripting working on an ArduPilot-flashed Crazyflie, there are a few necessary workarounds we need to employ.

One of the primary features of Crazyflie drones is their lightweight construction. The drones use the STM32 MCU as the primary flight controller which is a relatively low-power microcontroller with limited memory (1 Mb of flash).

For these limited boards, ArduPilot by default uses its “minimize_features.inc” script which cuts out all but the most essential base ArduPilot features. The first step is to enable scripting in your hardware definition file:
- In your development environment, navigate to the Crazyflie hwdef file:
```
path\...\libraries\AP_HAL_ChibiOS\hwdef\crazyflie2\hwdef.dat
```
- Near the bottom of the file, find the line that minimizes the ArduPilot features:
```
include ../include/minimize_features.inc
```
- Underneath this line, include the following line: 
```
define AP_SCRIPTING_ENABLED 1       #Lua Script Support
```
The next step is to create a new directory where we can store our custom Lua scripts. Ideally, we would use an external SD card to store Lua scripts so we could upload new scripts and make changes quickly and efficiently. Unfortunately, the external SD card functionality is quite memory intensive. As an aside, it is worth investigating in the future if it is possible to fit the SD card functionality on the STM32 Crazyflie MCU.

As a result, we are required to store our custom Lua scripts within the ArduPilot firmware itself on the STM32 MCU:
- Under the Crazyflie directory:
```
path\...\libraries\AP_HAL_ChibiOS\hwdef\crazyflie2
```
- Create a new folder named “scripts”. This folder is where we will upload our Lua scripts.

## Writing Custom Lua Scripts
There are many good resources online for how to write Lua scripts in ArduPilot, so this guide won’t go into too much detail here. Below are some links to get you started:

https://ardupilot.org/copter/docs/common-lua-scripts.html

https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/docs/docs.lua

### Basic Structure
We start by creating a new .lua file. We first define the initialization function at the beginning of the script which houses any one-time setup initialization you need:
```
function init()
  -- one-time setup (optional)
end
```
For simple periodic logic, we then implement an update() function that returns itself and a millisecond interval:
```
function update()
  -- your custom logic here
  return update, 1000  -- reschedule in 1000 ms
end
```
This interval defines the period at which you want your update function to repeat. Note that if you don’t want a function to repeat, simply remove this return statement.

We can also include a cleanup function here if needed, to be run when before the script finishes:
```
function uninit()
  -- cleanup on script termination (optional)
end
```
### Using the API
Inside your callbacks, you can use the functionality provided by the ArduPilot API. We can access things like the AHRS (Attitude and Heading Reference System) here to get state updates, the GCS (Ground Control Station) to send specific data to and from the drone, the servos to manually control the motors, etc.

Ex. Get current position (2D – xy), and move to a new positon:
```        
local pos = ahrs:get_position()
if pos then
    pos:offset(0, FORWARD_DIST)   -- north=0, east=+3 m → forward
    vehicle:set_target_location(pos)
    gcs:send_text(6,string.format("Goto +%dm",FORWARD_DIST)
else
    gcs:send_text(3,"No position")
    return nil
end
```

## Compiling & Flashing to the Crazyflie
After you have completed your custom Lua script and are ready to begin testing, we need to compile and flash the ArduPilot firmware. For detailed flashing instructions, please reference the [Compiling & Flashing Guide](compiling_and_flashing.md).

The main difference here to note is that the Lua scripting feature is not intended for lightweight MCU’s such as the STM32 found on the Crazyflie. ArduPilot only enables scripting by default on boards with at least 2 Mb of flash.

As a result, we may need to free up more space to meet the 1 Mb hardware memory limitation (especially if you have enabled other features such as Optical Flow, RangeFinder, SD Card, etc.). 

If you attempt to compile your firmware and get a build failed error:
```
Build failed -> task in 'bin/arducopter' failed (exit status 1)
```
Chances are you have exceeded the memory limit. Please reference the [Freeing up Memory Guide](freeing_up_memory.md) for detailed instructions on how to minimize the build size.

## Testing and Using Lua Scripts
Once you have successfully flashed your custom firmware with Lua scripting enabled and your custom Lua script saved, using the scripts is easy. Simply change the new parameter SCR_ENABLE from 0 to 1 upon startup of your drone.

After enabling scripting, your custom Lua scripts will immediately begin to run. If you did not hard code an exit condition, the scripts will run continuously until you disable scripting by changing SCR_ENABLE from 1 to 0.
