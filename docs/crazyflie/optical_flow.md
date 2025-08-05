# Optical Flow Guide
## What is Optical Flow?
ArduPilot ArduCopter supports a variety of optical flow sensors which estimate horizontal velocity.
Optical flow velocity estimation works by analyzing successive frames from a downward facing camera to detect how small image features (edges, corners or textured patches) shift from one frame to the next. The onboard flow algorithm computes a 2D pixel displacement field by matching these features across frames at a known frame rate.

![Optical Flow Principle](images/optical_flow/optical_flow_1.png)

*Gordon, Andrew. “Adventures in Optical Flow.” Technology, Thinking, Doing, 20 June 2021, www.andrewgordon.me/posts/Adventures-in-Optical-Flow/.*

Because the physical distance each pixel represents depends on the drone’s height above ground, the system also uses a Time of Flight (ToF) sensor’s vertical distance reading to establish a scale factor (approximately height / focal length).

Multiplying the measured pixel shifts by this scale and dividing by the time between frames yields the horizontal velocity of the vehicle relative to the ground. By continuously updating with each new image and height measurement, the autopilot obtains real time, drift corrected velocity estimates even in GPS denied environments.

The Crazyflie 2.1 supports the Bitcraze Flow Deck, which is an external expansion deck with a PWM3901 optical flow sensor and a VL53L1x ToF sensor (https://www.bitcraze.io/products/flow-deck-v2/).

This guide will start with getting the PWM3901 optical flow sensor working in ArduPilot. To do this, we need to make a few critical changes to the ArduPilot firmware.

As mentioned in previous guides, due to the lightweight construction of the Crazyflie drones, we need to enable certain additional features that have been disabled by default to save flash storage space. This is because the main MCU on the Crazyflie (STM32) has limited onboard flash of 1 Mb which is less than the minimum 2 Mb required for the full build of ArduPilot.

## PWM3901 in ArduPilot
To get the FlowDeck working in ArduPilot, we need to modify the base ArduPilot firmware to include a new driver for the PWM3901 sensor. 
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
define AP_OPTICALFLOW_ENABLED 1       #Optical Flow Support
```
This tells the compiler to include the optical flow library in our ArduPilot build regardless of the minimize features directive.

Next, we need to tell the HAL (Hardware Abstraction Layer) to use SPI:
- In the hwdef file above the minimize features line from earlier, find the line which disables ADC pins:
```
# no ADC pins
define HAL_USE_ADC FALSE
```
- Underneath this line, include the following:
```
define HAL_USE_SPI TRUE
```
This tells the HAL to initialize the SPI device backend which will allow us to register SPI devices.

In this respect, the next step is to add a new SPI device table in the hwdef file for our new optical flow sensor:
- In the hwdef file under the line which reserves flash space for the bootloader:
```
# reserve 32k for bootloader and 32k for flash storage
FLASH_RESERVE_START_KB 64
```
- Add a new section for SPI device definitions:
```
# SPI Device table
# Add FlowDeck Support
SPIDEV optflow     SPI1 DEVID0 E_CS1 MODE3 1*MHZ 1*MHZ
```
This line defines the optical flow sensor as an SPI device with the pointer “optflow”. It will allow us to access the hardware in our driver implementation later. 

### Modify the Optical Flow Library
Next, we need to add the new driver to the optical flow library:
- Start by navigating to the optical flow library:
```
path\...\libraries\AP_OpticalFlow\AP_OpticalFlow.cpp
```
- In the include statements at the top, add the following:
```
#include "AP_OpticalFlow_FlowDeck.h"
```
This adds our new driver to the optical flow frontend. In this same file we need to add a new case for our driver which corresponds to a new value for the optical flow parameter “FLOW_TYPE”:

- Navigate to the section where the cases are defined in the init() function:
```
void AP_OpticalFlow::init(uint32_t log_bit)
{
…
    switch ((Type)_type) {
    case Type::NONE:
        break;
    case Type::PX4FLOW:
#if AP_OPTICALFLOW_PX4FLOW_ENABLED
        backend = AP_OpticalFlow_PX4Flow::detect(*this);
#endif
        break;
…
```
- Add a new case for our new driver:
```
    case Type::FLOWDECK:    // Add Crazyflie FlowDeck support
#if AP_OPTICALFLOW_FLOWDECK_ENABLED
        hal.console->printf("AP_OpticalFlow::init trying FlowDeck detect\n"); // DEBUG
        hal.console->flush();
        backend = AP_OpticalFlow_FlowDeck::detect("optflow", *this);
#endif
```
Next, we need to add the new corresponding parameter value to the optical flow frontend header.
- Navigate to the frontend header file in the optical flow library:
```
path\...\libraries\AP_OpticalFlow\AP_OpticalFlow.h
```
- Find the parameter definition:
```
…
UPFLOW = 8, 
SITL = 10,
…
```
- And add the following:
```
…
UPFLOW = 8, 
FLOWDECK = 9,   <-- ADD THIS
SITL = 10,
```
We now need to modify the configuration file to include our new driver.
- Navigate to the config file in the optical flow library:
```
path\...\libraries\AP_OpticalFlow\AP_OpticalFlow_config.h
```
- At the end of the definitions, find the line:
```
…
#ifndef AP_OPTICALFLOW_UPFLOW_ENABLED #define AP_OPTICALFLOW_UPFLOW_ENABLED AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED
…
```
- And add:
```
…
#ifndef AP_OPTICALFLOW_FLOWDECK_ENABLED #define AP_OPTICALFLOW_FLOWDECK_ENABLED AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED
```
Finally, we are ready to add our new driver to the optical flow library.
- In the optical flow directory, add the attached driver [implementation](../../libraries/AP_OpticalFlow/AP_OpticalFlow_FlowDeck.cpp) and [header](../../libraries/AP_OpticalFlow/AP_OpticalFlow_FlowDeck.h) file (“AP_OpticalFlow_FlowDeck.cpp” and “AP_OpticalFlow_FlowDeck.h” respectively).
```
path\...\libraries\AP_OpticalFlow\
```
## VL53L1x in ArduPilot
The next step to getting the FlowDeck working in ArduPilot is to modify the base ArduPilot firmware to include a driver for the VL53L1x ToF sensor.

Please reference the [RangeFinder Guide](rangefinder.md) for detailed instructions.
## Compiling & Flashing to the Crazyflie
Before using the new optical flow driver in ArduPilot, we need to compile the custom firmware and flash it the Crazyflie. 

For detailed flashing instructions, please reference the [Compiling & Flashing Guide](compiling_and_flashing.md).

As with the Lua scripting feature, optical flow is not intended for lightweight MCU’s such as the STM32 found on the Crazyflie. ArduPilot only enables optical flow by default on boards with at least 2 Mb of flash. As a result, we may need to free up more space to meet the 1 Mb hardware memory limitation.

If you attempt to compile your firmware and get a build failed error:
```
Build failed -> task in 'bin/arducopter' failed (exit status 1)
```
Chances are you have exceeded the memory limit. Please reference the [Freeing up Memory Guide](freeing_up_memory.md) for detailed instructions on how to reduce the build size.

## Testing and Using Optical Flow
Once you have successfully flashed your custom firmware with optical flow enabled, using the flow deck is relatively simple. Start by changing the new parameter FLOW_TYPE from 0 to 9 upon startup of your drone.

After changing the parameter value and saving it to memory, restart the system by either cutting power to the drone directly or sending a reboot command through MAVLink.

Once the system has restarted, several new optical flow parameters should now be available. Ensure that all new optical flow parameters are set to zero:
```
FLOW_ADDR        0
FLOW_FXSCALER    0
FLOW_FYSCALER    0
FLOW_ORIENT_YAW  0
FLOW_POS_X       0.000000
FLOW_POS_Y       0.000000
FLOW_POS_Z       0.000000
```
After saving these new parameters, restart the system again.

The flow deck should now initialize properly upon powering the system. In your Ground Control Station of choice (ie. QGroundControl, MavProxy, etc.), verify the sensor is working properly by monitoring the live feed of the OPTICAL_FLOW parameter.

![QGC Optical Flow](images/optical_flow/optical_flow_3.png)

If the OPTICAL_FLOW parameter is present and the values are updated when the drone moves, the PWM3901 sensor is likely functioning properly.
