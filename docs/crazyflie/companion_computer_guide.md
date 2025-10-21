# Companion Computer Guide
## What is a Companion Computer?
A companion computer is an adjacent computer which operates alongside the main controller to provide additional compute headspace by offloading certain tasks. In the context of aerial drones, companion computers are often used to provide a platform for higher order compute tasks such as advanced localization using LIDAR, enabling onboard machine learning algorithms, automated mission command, etc. which are not suited for the flight controller.

## Companion Computer Use Case
For ArduSwarm, the additional resources of a companion computer are mainly leveraged to handle the high order automation for mission control and experimental algorithms. This functionality is extended to the user through a custom API which translates user commands to MAVLink messages which are sent over serial to the flight controller. The API can be accessed through a blank script which can be customized to fit the user's needs.

The companion computer is also used to facilitate a telemetry link between Crazyflie drones and a GCS (Ground Control Station) over WiFi.

## Bitcraze AI Deck
The AI Deck expansion board from Bitcraze is a great platform to enable the companion computer functionality described above. It combines the GAP8 arm-based processor and a ESP32 for lightweight, low-power computation ideal for embedded AI research and development on the micro-sized Crazyflie.

In terms of specs, the AI deck features:
- GAP8 - Ultra low power 8+1 core RISC-V MCU.
- 512 Mbit of flash.
- 64 Mbit of RAM.
- WiFi hosting via ESP32 (NINA-W102).
- Himax HM01B0 – Ultra low power 320×320 monochrome camera.

### Test AI Deck Compatibility
For ArduSwarm, we use the GAP8 processor for our automation via the API. To upload custom scripts we must flash the GAP8 over the air using Bitcraze tools with the GAP8 bootloader. Unfortunately, some of the AI decks from Bitcraze have an old version of the bootloader which does not support over the air flashing.

Thus, we need to test the functionality of the bootloader on the AI deck before proceeding. This process is detailed in the [official Bitcraze AI Deck Guide](https://www.bitcraze.io/documentation/tutorials/getting-started-with-aideck/).

Note that if your AI deck fails this test and will not flash, you can purchase a [Olimex ARM-USB-TINY-H JTAG](https://store.bitcraze.io/products/olimex-arm-usb-tiny-h-bundle) unit to manually re-flash the updated GAP8 bootloader.

## AI Deck with ArduPilot
To get the AI deck working within the ArduPilot ecosystem, we simply need to flash our custom firmware onto the AI deck (specifically the GAP8). 

If you plan on using pre-compiled firmware and don't plan on customizing the automation scripts, you can skip the next section.

### Developing for the AI Deck
Writing custom automation scripts is the heart of the ArduSwarm project. The ArduSwarm API provides a user-friendly sandbox environment to write scripts for swarming applications, automated mission sets and much more.

Please reference the [Using the API for Custom Automation](using_the_api_for_custom_automation.md) guide for detailed instructions of the development process.

## Flashing AI Deck
To flash firmware to the AI Deck, we need to use the Bitcraze provided tools for over the air flashing. This process requires the use of a Crazyflie drone flashed with the default Bitcraze firmware, a computer with the Bitcraze python library installed, and a [PA radio dongle](https://www.bitcraze.io/products/crazyradio-pa/).

### Prerequisites
If you have already flashed you Crazyflie drone with ArduPilot, please reference the [Restoring the Crazyflie](restoring_the_crazyflie.md) guide to restore the Crazyflie to the default Bitcraze firmware.

The next step is to install the Bitcraze python library on your computer. If you have already installed the library from previous guides, you can skip this step. If not, please reference the [official Bitcraze Client Installation Guide](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/installation/install/).

Once the Bitcraze library is installed, we need to enable the PA radio dongle for use with the library. Please reference the official Bitcraze Crazyradio guide:

- [MacOS/Linux](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/)
- [Windows](https://www.bitcraze.io/documentation/repository/crazyradio-firmware/master/building/usbwindows/)

### Flashing Guide
Once all of the above prerequisite requirements have been met, we can flash our custom firmware to the GAP8.

- Start by mounting the AI Deck onto your Crazyflie with default Bitcraze firmware.

- Power on the drone and wait until you see both green LEDs on the AI deck flashing.

- Plug in your PA radio dongle to your computer.

- Open a command terminal and navigate to the environment where the Bitcraze python library is located (if you installed the library in a python environment).

- In the command terminal, enter the following command:
'''
cfclient
'''

- The Bitcraze client application should open. 

- Connect your Crazyflie drone to your computer via USB.

- Press the "scan" button in the Crazyflie application, select the "usb://0" option.

![Client Scan](images/companion_computer_guide/client-scan.png)

- Press "connect" and then navigate to "Configure 2.x".

![Client Connect](images/companion_computer_guide/client-connect.png)

- In the configure window, record the radio channel, bandwidth, and address.

![Client Configure](images/companion_computer_guide/client-configure.png)

- Close the Bitcraze application.

- Find the local copy of your AI deck firmware. If you wish to use pre-compiled firmware, please download [here](compiled_firmware).

- In the terminal, enter the following command to flash the AI deck over the air:
'''
cfloader flash {path/to/your/local/firmware}/target.board.devices.flash.img deck-bcAI:gap8-fw -w radio://0/100/2M/E7E7E7E706
'''

Where the radio should match the channel, bandwidth, and address of your specific drone.

The drone will proceed to flash the AI deck over the air. The drone will restart several times during the process.

If you came from the Quick Start Guide, return to the guide [here](quick_start_guide.md). Otherwise, proceed to the next section.

## Testing and Using the AI Deck
Now that the AI deck is flashed with custom firmware, we can use it within the ArduSwarm platform. If you need to flash ArduPilot onto your drone after restoring it to default firmware in this guide, please reference the [Compiling and Flashing Guide](compiling_and_flashing.md).

### Setup
Once you have attached the AI deck to your Crazyflie which has ArduPilot flashed on it, power on the drone and connect the drone to a GCS. Update the following parameters:

'''
SERIAL1_PROTOCOL 2
SERIAL1_BAUD     115
SERIAL2_PROTOCOL 2
SERIAL2_BAUD     1000
SERIAL3_PROTOCOL 2
'''

Reset the drone by powering the Crazyflie on/off. The Crazyflie should now be able to communicate with the AI deck over serial.

### Telemetry Link
If you flashed the [mavbridge firmware](compiled_firmware/ai_deck/mavbridge/) onto the AI deck to use the AI deck as a telemetry link, there are a couple extra steps to get telemetry working with your GCS of choice.

- Open your GCS of choice (ex. QGroundControl).

- Open the connection settings in your GCS.

![GCS WiFi Settings](images/companion_computer_guide/gcs-settings.png)

- Create a new connection link with the following parameters:

![GCS UDP Setup](images/companion_computer_guide/gcs-wifi.png)

Next, download a local copy of the [mavbridge python script](python/mavbridge.py). This script will forward the telemetry link being sent over WiFi from the AI deck to your GCS.

Next, connect to the WiFi network "AIDeckDebugAP".

- Then, run the python script through a command terminal with the following command:
'''
python /path/to/local/copy/mavbridge.py
'''

Note that this script requires the following python libraries as prerequisites:
- socket
- struct
- threading
- time

Finally, open your GCS of choice. If you set up the UDP connection correctly, the GCS should automatically connect to your Crazyflie's telemetry stream. You can now use the drone regularly in ArduPilot.

### Custom Mission
If you flashed a custom mission onto the AI deck (ex. [voodoo](compiled_firmware/ai_deck/voodoo/)), using the AI deck depends on how you have configured your custom firmware.

Most firmware use the wifi_mission_control() function to manage the mission through user commands over WiFi. To use this firmware, we need to connect to the WiFi access point created by the AI deck and send commands.

First, download a local copy of the [WiFi Command python script](python/wifi_command.py). This script will connect send user commands to the AI deck and receive messages sent by the AI deck.

Next, connect to the WiFi network "AIDeckDebugAP".

- Then, run the python script through a command terminal with the following command:
```
python /path/to/local/copy/wifi_command.py
```

Note that this script requires the following python libraries as prerequisites:
- socket
- struct
- threading
- time

You can now manage you custom mission by sending commands. Depending on how you have set up your firmware, sending certain commands will do things like start the mission, reset the mission, cancel the mission, etc.