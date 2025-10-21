# Logging Guide
## Logging Use Case
Logging is an essential capability for any aerial drone system. Logging systems poll the main flight controller and adjacent sensors at set intervals and record the responses in memory for later analysis. This data is used for critical tasks like tuning various controllers, analyzing flight performance, and observing mission history. 

## Detailed vs. Telemetry Logs
ArduPilot provides the functionality to record both detailed and telemetry logs, depending on the use case. The distinctive difference between these options is the maximum bandwidth possible. 

Telemetry logs send polled data over the air from the drone's flight controller to any receiving GCS (Ground Control Station). This provides the user with a real-time recording of the drone, but comes with a few key drawbacks:

- The maximum bandwidth is on the order of kilobytes per second (around 5 to 10 kB/s).
- The drone must maintain its connection to the receiving GCS at all times to avoid holes in the logs.

Detailed or "Dataflash" logs directly record the polled data to onboard flash for future analysis. As the name suggests, the increased bandwidth of a direct physical connection from the flight controller to the storage (around 0.5 to 1 megabyte per second) allows for a significant increase in resolution of logs. This system results in a few drawbacks:

- Logs cannot be accessed in real time. Logs must be downloaded after the flight has concluded.
- The SD Card Deck adds around 2g of additional weight, eating into the relatively small maximum payload of the Crazyflie 2.1 (15g).

## Enabling Telemetry Logging
Enabling telemetry logs on the ArduSwarm platform requires no modifications to the base ArduPilot firmware.

This guide will not cover the specifics on how to enable these logs, but if you are interested please reference the [official ArduPilot Telemetry Log Guide](https://ardupilot.org/planner/docs/mission-planner-telemetry-logs.html).

## Enabling Detailed Logging
To enable detailed logging in ArduPilot on the Crazyflie 2.1 drones, we need to make a few changes to the base ArduPilot firmware.

### Modify the HWDEF file
Start by enabling the FAT filesystem in the hardware definition file:

- In your development environment, navigate to the Crazyflie hwdef file:
```
path\...\libraries\AP_HAL_ChibiOS\hwdef\crazyflie2\hwdef.dat
```
- Near the bottom of the file, find the line that minimizes the ArduPilot features:
```
include ../include/minimize_features.inc
```
- Above this line, include the following line: 
```
define HAL_OS_FATFS_IO 1      #Detailed Logging Support
```
This tells the compiler to include the filesystem library in our ArduPilot build regardless of the minimize features directive.

Next, we need to tell the HAL (Hardware Abstraction Layer) to use SPI. If you have already enabled SPI for the optical flow driver, you can skip this step.
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

In this respect, the next step is to add a new SPI device table in the hwdef file for our sd card deck:
- In the hwdef file under the line which reserves flash space for the bootloader:
```
# reserve 32k for bootloader and 32k for flash storage
FLASH_RESERVE_START_KB 64
```
- Add a new line to define the sd card deck as an SPI device:
```
# SPI Device table
# Add FlowDeck Support
SPIDEV optflow     SPI1 DEVID0 E_CS1 MODE3 1*MHZ 1*MHZ
SPIDEV sdcard      SPI1 DEVID1 E_CS0 MODE0 400*KHZ 25*MHZ  <-- ADD THIS
```
This line defines the sd card deck as an SPI device with the pointer “sdcard”. It allows the HAL to access the hardware in the filesystem driver.

### Compiling & Flashing to the Crazyflie
Before using the sd card in ArduPilot, we need to compile the custom firmware and flash it the Crazyflie.

For detailed flashing instructions, please reference the [Compiling & Flashing Guide](compiling_and_flashing.md).

If you attempt to compile your firmware and get a build failed error:
```
Build failed -> task in 'bin/arducopter' failed (exit status 1)
```
Chances are you have exceeded the memory limit. Please reference the [Freeing up Memory Guide](freeing_up_memory.md) for detailed instructions on how to reduce the build size.

### Formatting SD Card
ArduPilot's filesystem expects the external SD card to be formatted FAT. To proceed, please format your SD card with a single FAT partition.

- MacOS - [Disk Utility](https://support.apple.com/en-ca/guide/disk-utility/welcome/mac)
- Windows - [Disk Management](https://support.microsoft.com/en-us/windows/disk-management-in-windows-ad88ba19-f0d3-0809-7889-830f63e94405)
- Linux - [GParted](https://gparted.org/)

## Testing and Using Optical Flow
Once you have successfully flashed your custom firmware with SD card support enabled, using detailed logging is relatively simple.

Upon startup of the Crazyflie, change the parameter LOG_BACKEND_TYPE from 0 to 1. This will enable the SD card logging backend.

If you desire logging when power is applied instead of when the drone is armed, set the parameter LOG_DISARMED from 0 to 1.

Finally, you need to update the parameter LOG_BITMASK. The default value 0 disables all logging. Use your GCS of choice to choose which parameters you would like to log.

For further details, please reference the [official ArduPilot Dataflash Log Guide](https://ardupilot.org/copter/docs/common-downloading-and-analyzing-data-logs-in-mission-planner.html).