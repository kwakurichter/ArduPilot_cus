# RangeFinder Guide
## What is ToF?
Time of flight (ToF) rangefinders like the VL53L1x work by emitting short, nanosecond scale pulses of invisible 940 nm laser light and then timing how long it takes for those pulses to bounce off a target and return to the sensor’s SPAD (single photon avalanche diode) detector.

![RangeFinder Principle](images/rangefinder/rangefinder_2.png)

*Craig, R. “Time-of-Flight Camera.” Wikipedia, Wikimedia Foundation, 16 May 2021, en.wikipedia.org/wiki/Time-of-flight_camera.*

Inside the VL53L1x module, ST’s FlightSense firmware on a tiny onboard microcontroller precisely timestamps the emission and reception events with picosecond resolution. By applying the known speed of light to the measured round trip time, the sensor computes an absolute distance (up to 4 m at rates up to 50 Hz), independent of ambient lighting or target reflectivity.

The Crazyflie 2.1 supports the Bitcraze FlowDeck, which is an external optical flow expansion deck that relies on accurate altitude estimates from the VL53L1x (https://www.bitcraze.io/products/flow-deck-v2/).

This guide will focus on getting the VL53L1x ToF sensor working in ArduPilot. To do this, we need to make a few critical changes to the ArduPilot firmware.

As mentioned in previous guides, due to the lightweight construction of the Crazyflie drones, we need to enable certain additional features that have been disabled by default to save flash storage space. This is because the main MCU on the Crazyflie (STM32) has limited onboard flash of 1 Mb which is less than the minimum 2 Mb required for the full build of ArduPilot.

## VL53L1x in ArduPilot
Previous builds of ArduPilot already contain a driver for the VL53L1x sensor however, this legacy implementation manually initializes and handles the sensor which does not work with the FlowDeck VL53L1x for some unknown reason.

Instead of attempting to fix the manual implementation, we include the API directly from the manufacturer to handle the low-level device operation. This provides a more robust, isolated driver implementation.

To do this, we need to modify the existing VL53L1x driver in the legacy base ArduPilot firmware.
### Modify the HWDEF file
Start by enabling optical flow in the hardware definition file:

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
define AP_RANGEFINDER_ENABLED 1
```
This tells the compiler to include the range finder library in our ArduPilot build regardless of the minimize features directive.

### Modify the Range Finder Library
Next, we need to modify the existing driver in the range finder library:

- Start by navigating to the legacy VL53L1x driver:
```
path\...\libraries\AP_RangeFinder\AP_RangeFinder.cpp
```
- Find the initializations of the drivers:
```
#if AP_RANGEFINDER_VL53L1X_ENABLED
```
- Remove the existing if statement here and replace it with the following:
```
if (_add_backend(AP_RangeFinder_VL53L1X::detect(state[instance], params[instance],
                                                hal.i2c_mgr->get_device(i, params[instance].address)), // <-- Only 3 arguments now
                    instance)) {
    break;
}
```
This simplifies the driver implementation as we hard-code the sensor distance mode rather than passing a separate parameter. Note that we may revert in the future if necessary.

The next step is to replace the existing driver [implementation](../../libraries/AP_RangeFinder/AP_RangeFinder_VL53L1X.cpp) and [header](../../libraries/AP_RangeFinder/AP_RangeFinder_VL53L1X.h) files with the attached files.

- Replace the following implementation and header file respectively:
```
path\...\libraries\AP_RangeFinder\AP_RangeFinder_VL53L1X.cpp

path\...\libraries\AP_RangeFinder\AP_RangeFinder_VL53L1X.h
```
As mentioned previously, the new driver uses the ST Microelectronics API directly instead of manually handling the device. The next step is thus to add the [3rd Party API](../../libraries/vl53l1x_api) to the ArduPilot firmware.

- Navigate to the libraries folder:
```
path\...\ardupilot\libraries
```
- Add the attached “vl53l1x_api” folder and all of its contents to the libraries directory.

This provides the driver with the low-level functions and backend infrastructure that are used to handle the sensor. These API files are up to date as of May 2025. Note that the API may change with future updates.

ArduPilot is not built to handle 3rd party libraries as-is. The base ArduPilot firmware does not link the API files when compiling, so we need to make a few more critical changes.

- Navigate to the main ArduPilot build script:
```
path\...\ardupilot\wscript
```
- Find the “build()” function definition, and under the following line:
```
bld.get_board().build(bld)
```
- Add the following:
```
bld.recurse('libraries/vl53l1x_api')    # Add Crazyflie flowdeck support via ST vl53l1x API
```
This line tells the compiler to recursively search the libraries folder for our 3rd party API by name. Without this line, the compiler will not find and initialize the API.

It should be noted that this is a patch solution and should be cleaned up in the future.

The next step is to add a similar hard-coded instruction to search for our 3rd party API at the lower ArduCopter level.

- Navigate to the ArduCopter build script:
```
path\...\ardupilot\ArduCopter\wscript
```

- Under the following line:
```
vehicle = bld.path.name
```
- Add the following block of code:
```
bld.env.append_value(
    'INCLUDES',
    [   
        'libraries/vl53l1x_api/core/inc',
        'libraries/vl53l1x_api/platform/inc',
    ],
)
```
- Then, in the next block of code, add the following:
```
bld.ap_stlib(
    name=vehicle + '_libs',
    ap_vehicle=vehicle,
    ap_libraries=bld.ap_common_vehicle_libraries() + [
        'AC_AttitudeControl',
        'AC_InputManager',
    …
        'AP_KDECAN',
        'AP_SurfaceDistance',
        'vl53l1x_api',     			<-- ADD THIS
    ],
)
```
The compiler will now properly link the 3rd party API with our updated driver. Calling functions, accessing classes, etc. is now possible. 
## Compiling & Flashing to the Crazyflie
Before using the new range finder driver in ArduPilot, we need to compile the custom firmware and flash it the Crazyflie. For detailed flashing instructions, please reference the [Compiling & Flashing Guide](compiling_and_flashing.md).

Range finders are not intended for lightweight MCU’s such as the STM32 found on the Crazyflie. ArduPilot only enables range finders by default on boards with at least 2 Mb of flash. As a result, we may need to free up more space to meet the 1 Mb hardware memory limitation.

If you attempt to compile your firmware and get a build failed error:
```
Build failed -> task in 'bin/arducopter' failed (exit status 1)
```
Chances are you have exceeded the memory limit. Please reference the [Freeing up Memory Guide](freeing_up_memory.md) for detailed instructions on how to minimize the build size.

## Testing and Using ToF
Once you have successfully flashed your custom firmware with range finders enabled, using the flow deck is relatively simple. Start by changing the parameter “RNGFNDX_TYPE” from 0 to 16 upon startup of your drone.

After changing the parameter value and saving it to memory, restart the system by either cutting power to the drone directly or sending a reboot command through MAVLink.

Once the system has restarted, several new range finder parameters should now be available. Ensure that all new range finder parameters are set to match the following:
```
RNGFNDX_ADDR     41
RNGFNDX_FUNCTION 0
RNGFNDX_GNDCLR   0.100000
RNGFNDX_MAX      7.000000
RNGFNDX_MIN      0.200000
RNGFNDX_OFFSET   0.000000
RNGFNDX_ORIENT   25
RNGFNDX_PIN      -1
RNGFNDX_POS_X    0.000000
RNGFNDX_POS_Y    0.000000
RNGFNDX_POS_Z    0.000000
RNGFNDX_PWRRNG   0
RNGFNDX_RMETRIC  1
RNGFNDX_SCALING  3.000000
RNGFNDX_STOP_PIN -1
RNGFNDX_TYPE     16
```
After saving these new parameters, restart the system again.

The ToF sensor on the flow deck should now initialize properly upon powering the system. In your Ground Control Station of choice (ie. QGroundControl, MavProxy, etc.), verify the sensor is working properly by monitoring the live feed of the RANGEFINDER parameter.

![QGC RangeFinder](images/rangefinder/rangefinder_1.png)

If the RANGEFINDER parameter is present and the values are updated when the drone moves, the VL53L1x sensor is likely functioning properly.
