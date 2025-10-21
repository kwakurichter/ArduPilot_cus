# Compiling & Flashing Guide
Compiling and flashing custom ArduPilot firmware to the Crazyflie drones is necessary to enable to the full Bitcraze feature set, and is also very useful for modifying the base firmware to better suit your needs.

This process varies significantly depending on your specific hardware and software setup.  This guide will not go over specifics needed to setup the build environment on your platform (Windows, Linux, MacOS). Instead, please follow the below guides for your platform of choice:

Windows:
https://ardupilot.org/dev/docs/building-setup-windows.html

Linux:
https://ardupilot.org/dev/docs/building-setup-linux.html

MacOS:
https://ardupilot.org/dev/docs/building-setup-mac.html

## Compiling the Firmware
Once you have setup your build environment and have customized the firmware we can move on to compiling the firmware. 
- Start by opening a terminal.
- If you are using a virtual environment, ensure your Python interpreter has access to the required prerequisites.
- Navigate to your custom ardupilot repository:
```
path\...\ardupilot
```
- It is good practice to clean your build objects at this time (in case some submodules have been updated):
```
./waf clean
```

- Next, configure your build to the Crazyflie board:
```
./waf configure --board crazyflie2 BL=1
```
As an aside, it is crucial that you compile your firmware with a bootloader (the “BL=1” part in the above line) so we can easily re-flash the Crazyflie with new firmware or restore the Crazyflie to factory settings if necessary.
- Finally, build ArduCopter:
```
./waf copter
```

Note that if the build fails for whatever reason, and you are attempting to debug the issue, you can tell the compiler to increase the verbosity to better monitor the compiler:
```
./waf copter -v
```
A common build failure error you may encounter is related to the limited flash memory available on the STM32 MCU. If you attempt to compile your firmware and get the following error:
```
Build failed -> task in 'bin/arducopter' failed (exit status 1)
```
Chances are you have exceeded the memory limit. Please reference the [Freeing up Memory Guide](freeing_up_memory.md) for detailed instructions on how to the build size.

## Flashing the Firmware
If the compiler was successful and you have successfully built your custom ArduCopter firmware, we can now proceed to flashing the firmware onto the Crazyflie drone.

### Retrieve the Hex Code
The most robust way to flash custom firmware onto a Crazyflie drone is to manually retrieve the compiled hex code.

- Navigate to the build folder of your custom ArduPilot repository:
```
path\...\ardupilot\build\crazyflie2\bin
```
- The build we want should match the following:

“arducopter_with_bl.hex”

### Install the Flashing Software
Because the Crazyflie drones use the STM32 microcontroller as their flight controller, we will use the flashing software directly from the manufacturer to flash the MCU with our custom firmware. You can find the flashing software from the link below.

STM32CubeProgrammer:
https://www.st.com/en/development-tools/stm32cubeprog.html

Install STM32CubeProgrammer and open it before moving on to the next step.

### Prepare the Crazyflie for Flashing
Before we can flash the firmware to the Crazyflie drone, we need to manually put the drone in bootloader mode.

- Disconnect all power sources (remove battery, unplug usb cable).
- Press and hold the power button. While holding the power button:
    - Plug the usb cable into the drone.
    - The M2 LED should begin blinking slowly. Keep holding the power button until the M2 LED begins to blink faster (around 3 seconds).
    - Release the power button.

    <video width="600" controls>
      <source src="images/compiling_and_flashing/bootloader-second.mp4" type="video/mp4">
    </video>

### Flash the Custom Firmware
With STM32CubeProgrammer open and the Crazyflie in bootloader mode, we start by connecting the drone to the flashing software.

- Change the connection type from UART to USB in the drop-down menu:
![USB Selection](images/compiling_and_flashing/compiling_and_flashing_1.png)
- Search for devices and ensure a usb device is found:
![USB Search](images/compiling_and_flashing/compiling_and_flashing_2.png)
- Next, connect to the drone:
![USB Connect](images/compiling_and_flashing/compiling_and_flashing_3.png)
- Once the drone has connected, press the “Open file” button:
![Open File](images/compiling_and_flashing/compiling_and_flashing_4.png)
- Navigate to your compiled firmware .hex file from earlier and open it:
![Select File](images/compiling_and_flashing/compiling_and_flashing_5.png)
- Once opened, press the “Download” button to flash the firmware:
![Flash Crazyflie](images/compiling_and_flashing/compiling_and_flashing_6.png)
- After the firmware is finished flashing, disconnect the usb from the Crazyflie.
- Turn on the Crazyflie after restoring power to the drone.

Your Crazyflie drone is now flashed with ArduPilot. Note that flashing the drone with ArduPilot does not affect the Bitcraze NRF51 firmware that is present on the secondary NRF51 microcontroller.

Also, because we compiled our ArduPilot firmware with a bootloader earlier, we can re-flash the Crazyflie with a new custom firmware at any time by first putting the drone into bootloader mode as described previously and following the same instructions.

If you need to restore the Crazyflie to the factory Bitcraze firmware for any reason, follow the instructions in the [Restoring the Crazyflie Guide](restoring_the_crazyflie.md).
