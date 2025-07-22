# Freeing up Memory Guide
## Why?
One of the primary features of Crazyflie drones is their lightweight construction. The drones use the STM32 MCU as the primary flight controller which is a relatively low-power microcontroller with limited memory (1 Mb of flash). ArduPilot by contrast is primarily designed for more capable boards with a minimum of 2 Mb of onboard flash.

For these limited boards, ArduPilot by default uses its “minimize_features.inc” script which cuts out all but the most essential base ArduPilot features. For all but the most basic applications using the bare the Crazyflie hardware, we need to enable features which have been disabled by the “minimize_features” function.

Additionally, modifying the base ArduPilot firmware can quickly eat up flash storage. In these cases, selecting additional features to cut from your custom build may be required to successfully compile your firmware.
## When?
If you notice your custom build will exceed the 1 Mb memory limit, you will need to consider trimming ArduPilot to size. This limitation will most likely present itself when you attempt compiling your custom firmware and get a build failed error:
```
Build failed -> task in 'bin/arducopter' failed (exit status 1)
```
Chances are you have exceeded the memory limit. I will go over some things you can do to further reduce memory used to hopefully get your firmware compiling.

## Free up Memory
- Navigate to the ArduCopter configuration file:
```
path\...\ardupilot\ArduCopter\APM_Config.h
```
This file lists several core features which can be disabled. To re-iterate, these features are generally considered core features and thus should only be disabled if absolutely necessary.
- Any feature you wish to disable, simply uncomment the line:
```
Ex.    //#define NAV_GUIDED    0  -->  Change to:   #define NAV_GUIDED    0
```
The approximate size of each of these features is listed in the comments of the configuration file. 
If you disable all of the features that can reasonably be disabled and are still running into memory limits, you will need to find a way to reduce the space used by your custom features.
