# Quick Start Guide
## Why?
The ArduSwarm platform involves a fair amount of modification to both the base ArduPilot firmware and base Bitcraze firmware. If you are not interested in modifying the firmware yourself or you do not have the coding skills to do the modifications yourself, this quick start guide will allow you to implement the ArduSwarm platform using pre-compiled firmware.

## Prerequisites
To begin the process, we need to gather the required Bitcraze hardware and install required the software tools. The hardware requirements are as follows:

### Parts List
- (Qty. 1+) Crazyflie 2.1 Kit
- (Qty. 1+) Flow Deck v2
- (Qty. 1+) SD Card Deck
- (Qty. 1+) Micro SD card (any brand)
- (Qty. 1+) AI Deck
- (Qty. 1) Crazyradio PA dongle
- (Qty. 1) Micro USB Cable

To ensure you have all of the required hardware, please reference the [Hardware Setup](hardware_setup.md) guide.

### Software Tools
For the software tools, we will start by installing the Bitcraze software. 

Please reference the [official Bitcraze Client Installation Guide](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/installation/install/) to install the Bitcraze client (cfclient) and python library (cflib) on your computer.

You can verify the installation was successful by opening a command terminal and navigating to where the software was installed.

![Command Terminal](images/quick_start_guide/terminal.png)

Next, enter the following commands individually to verify the tools are installed:
```
cloader
cfclient
```

| ![CLoader Tool](images/quick_start_guide/cfloader.png) | ![CFClient Application](images/quick_start_guide/cfclient.png) |
|--------------------------|--------------------------|

You should see the cloader tool in the command terminal and the cfclient application should open.

Next, we need to download and install the STM32CubeProgrammer software which will be used to flash ArduPilot onto the Crazyflie drone (STM32).

You can find the flashing software from the link below.

[STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)

Install STM32CubeProgrammer and open it before moving on to the next step.

![STM32CubeProgrammer](images/quick_start_guide/stm32programmer.png)

## Flashing the Firmware
Flashing all of the required firmware must be done according to a strict order of operations. This is because the ArduSwarm firmware is inter-dependant and certain hardware - the AI Deck and the secondary NRF51 MCU (Micro-Controller Unit) - require Bitcraze tools to flash.

If your Crazyflie drone has previously been flashed with ArduPilot, please reference the [Restoring the Crazyflie](restoring_the_crazyflie.md) guide before proceeding.

### AI Deck
Within ArduSwarm, the AI Deck serves as the "brain" of each drone, acting as a companion computer to the main STM32 flight controller. The AI Deck houses all of the custom missions and automation that are at the heart of the ArduSwarm platform.

To flash the firmware on the AI deck, please reference the [Companion Computer Guide](companion_computer_guide.md) and return to this quick start guide once you have successfully flashed one of the pre-compiled example firmware.

### NRF51 Secondary MCU
The NRF51 is the secondary MCU which is located on the base Crazyflie drones. It handles secondary tasks such as communication via 2.4 GHz radio and power management. To enable P2P (Peer-to-Peer) communication between drones for swarming and battery state updates within ArduSwarm, we need to flash a custom version of the official Bitcraze firmware onto the NRF51.

You can find a pre-compiled version of this firmware [here](compiled_firmware/nrf51/cf2_nrf.bin).

- Start by disconnecting the AI Deck from your drone.

- Disconnect the battery and usb cable from the drone.

- While holding down the power button, plug in the usb cable.

    - The blue LED (labelled M2) on the drone should start to blink, release the power button as soon as it does.

    <video width="600" controls>
      <source src="images/quick_start_guide/bootloader-first.mp4" type="video/mp4">
    </video>

    - Two blue LEDs (M2 and M3) should start blinking now. If only one is blinking, disconnect power from the drone and try again.

- Plug in your [PA radio dongle](https://www.bitcraze.io/products/crazyradio-pa/) which was used previously to flash the AI Deck.

- Open a command terminal and use the following command to flash the NRF51 radio:
```
cfloader flash path-to-your-folder/cf2_nrf.bin nrf51-fw
```

The NRF51 should now flash over the air. It should only take a few seconds. You will know if you were successful if the Crazyflie eventually powers on again by itself normally.

![Flashing the NRF](images/flashing_the_nrf/flashing.png)

### STM32 Flight Controller
The STM32 is the main MCU on the Crazyflie drones which functions as the flight controller. The flight controller is responsible for managing all of the primary flight related tasks, including controlling/stabilizing the motors, polling sensors for data, state estimation, interpreting commands, managing telemetry communication, etc.

The final software piece of the platform is to flash the STM32 with a custom version of ArduPilot which supports ArduSwarm.

A pre-compiled version of this firmware can be found [here](compiled_firmware/ardupilot/arducopter_with_bl.hex).

***Prepare the Crazyflie for Flashing***

Before we can flash the firmware to the Crazyflie drone, we need to manually put the drone in bootloader mode.

- Disconnect all power sources and expansion decks (remove battery, unplug usb cable).
- Press and hold the power button. While holding the power button:
    - Plug the usb cable into the drone.
    - The M2 LED should begin blinking slowly. Keep holding the power button until the M2 LED begins to blink faster (around 3 seconds).
    - Release the power button.
        <video width="600" controls>
          <source src="images/compiling_and_flashing/bootloader-second.mp4" type="video/mp4">
        </video>

***Flash the Custom Firmware***

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

Your Crazyflie drone is now flashed with ArduPilot. Note that flashing the drone with ArduPilot does not affect the NRF51 firmware we flashed previously.

If you need to restore the Crazyflie to the factory Bitcraze firmware for any reason, follow the instructions in the [Restoring the Crazyflie Guide](restoring_the_crazyflie.md).

## Next Steps
Now that all of the ArduSwarm firmware is installed, we can move on to flight testing. Re-assemble the drone according to the [Hardware Setup](hardware_setup.md) guide.

Once assembled, proceed to the[Pre-Flight Checklist](pre_flight_checklist.md).