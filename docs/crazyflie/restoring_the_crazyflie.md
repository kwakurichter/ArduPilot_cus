# Restoring the Crazyflie Guide
## Why?

There are multiple reasons why you may want to restore an ArduPilot-flashed Crazyflie to the factory Bitcraze firmware and settings. For example, your ArduPilot firmware may be corrupted or you might simply want to use the Crazyflie as originally intended again.

Whatever the reason, the process of re-flashing a Crazyflie with the Bitcraze flight control software is relatively simple.

## Flashing Bitcraze Firmware
The simplest way to restore the Crazyflie is to use the software tools provided by Bitcraze and flash the drone over the air. Note that you will need a Crazyflie PA Radio for this process.

### Install CFClient
The first step is to download and install the Bitcraze Crazyflie software, cfclient. To install the software on your platform of choice, follow the instructions in the link below:

https://github.com/bitcraze/crazyflie-clients-python/blob/master/docs/installation/install.md
 
### Download the Loader Tool
After installing cfclient, we need to download the Crazyflie loader tool used to restore corrupted Crazyflies. You can find the file at the link below:

https://github.com/bitcraze/crazyflie2-stm-bootloader/releases

Download the “cf2loader-1.0.bin” file.

### Install the Flashing Software
Currently, the cfclient software will not recognize the ArduPilot-flashed Crazyflie. We need to use the loader tool to restore the Crazyflie to a state where it is recognizable first.

Because the Crazyflie drones use the STM32 microcontroller as their flight controller, we will use the flashing software directly from the manufacturer to flash the MCU with the loader tool. You can find the flashing software from the link below.

STM32CubeProgrammer:
https://www.st.com/en/development-tools/stm32cubeprog.html

Install STM32CubeProgrammer and open it before moving on to the next step.

### Prepare the Crazyflie for Flashing
Before we can flash the loader tool to the Crazyflie drone, we need to manually put the drone in bootloader mode.

- Disconnect all power sources (remove battery, unplug usb cable).
- Press and hold the power button. While holding the power button:
    - Plug the usb cable into the drone.
    - The M2 LED should begin blinking slowly. Keep holding the power button until the M2 LED begins to blink faster (around 3 seconds).
    - Release the power button.
    <video width="600" controls>
      <source src="images/compiling_and_flashing/bootloader-second.mp4" type="video/mp4">
    </video>    

### Flash the Loader Firmware
With STM32CubeProgrammer open and the Crazyflie in bootloader mode, we start by connecting the drone to the flashing software.

- Change the connection type from UART to USB in the drop-down menu:
![USB Selection](images/restoring_the_crazyflie/restoring_the_crazyflie_1.png)
- Search for devices and ensure a usb device is found:
![USB Search](images/restoring_the_crazyflie/restoring_the_crazyflie_2.png)
- Next, connect to the drone:
![USB Connect](images/restoring_the_crazyflie/restoring_the_crazyflie_3.png)
- Once the drone has connected, press the “Open file” button:
![Open File](images/restoring_the_crazyflie/restoring_the_crazyflie_4.png)
- Navigate to the loader tool from earlier and open it:
![Select File](images/restoring_the_crazyflie/restoring_the_crazyflie_5.png)
- Once opened, press the “Download” button to flash the firmware:
![Flash Crazyflie](images/restoring_the_crazyflie/restoring_the_crazyflie_6.png)
- After the tool is finished flashing, disconnect the usb from the Crazyflie.

### Restore Crazyflie Firmware over Radio
- Open a terminal and start the cfclient software by entering the following command:
![Open Cfclient](images/restoring_the_crazyflie/restoring_the_crazyflie_7.png)
- Once the client is open, put the drone into bootloader mode:
    - Disconnect all power sources (remove battery, unplug usb cable).
    - Press and hold the power button. While holding the power button:
        - Plug the usb cable into the drone.
        - Once the M2 LED starts flashing, release the power button immediately.
- In the cfclient, navigate to the “Connect” and then “Bootloader” button:
![Open Bootloader](images/restoring_the_crazyflie/restoring_the_crazyflie_8.png)
- In the bootloader window, press the “Cold boot (recovery)” button:
![Open Cold Boot](images/restoring_the_crazyflie/restoring_the_crazyflie_9.png)
- Insert the Crazyflie PA Radio into your computer.
- Ensure that the “cf2” option is selected:
![Select cf2](images/restoring_the_crazyflie/restoring_the_crazyflie_10.png)
- Once the “Status” in the bottom left-hand corner changes to “connected”, press the “Program” button in the bottom right-hand corner.
![Flashing](images/restoring_the_crazyflie/flashing.png)

Cfclient will now re-flash the Bitcraze firmware onto the Crazyflie over the air.

