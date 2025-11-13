# Pre-Flight Checklist
If you have reached this point with your ArduSwarm drones, it means you have:

- Flashed all of the custom firmware on the:
    - [AI Deck](companion_computer_guide.md).
    - [NRF51 MCU](crazyradio.md).
    - [STM32 MCU](compiling_and_flashing.md).
- Assembled all of the required [hardware](hardware_setup.md):
    - Crazyflie 2.1.
    - Flow Deck v2.
    - SD Card Deck.
    - Micro SD card (any brand).
    - AI Deck.

Congratulations! You are almost ready for the first flight!

This final checklist will ensure your first flight doesn't end in disaster.

## Checklist
1. Inspect the propellers on your drone.
- Make sure there the propellers are not damaged. Large bends or cracks can cause performance issues which can lead to instability.

[Placeholder]

- Ensure your propellers are correctly installed. 
    - The leading edge of each propeller (the higher edge) must be facing the same direction as the direction of travel of each motor.
    - On quadcopters like the Crazyflie, opposing motors travel in the same direction.

| ![Prop Direction](images/pre_flight_checklist/prop-driection.JPG) | ![Prop Orientation](images/pre_flight_checklist/prop-orientation.JPG) |
|--------------------------|--------------------------|

2. Check the position of the battery.
- Make sure the battery is positioned in the center of the drone and not hanging off to one side.

![Battery Position](images/pre_flight_checklist/battery-position.JPG)

3. Make sure the AI Deck is functioning properly.
- Check if both green LEDs on the top of the AI Deck are blinking.
- Check if the WiFi access point from the AI Deck is visable.
- If either of these conditions are not met, check if the reset pin is touching the bottom of the AI Deck, described [here](hardware_setup.md).
- If so, push the exposed pin down and try restarting the drone.

[Placeholder]

4. Upload the parameters to the drone.
- Turn on your assembled drone and plug it into your computer via the micro USB cable.
- Open your GCS (Ground Control Station) of choice.
- Download the default ArduSwarm parameters [here](parameters/default.params).
- Navigate to the parameter tool in your GCS.

| ![QGC Parameters (1)](images/pre_flight_checklist/qgc-parameters-1.png) | ![QGC Parameters (2)](images/pre_flight_checklist/qgc-parameters-2.png) | ![QGC Parameters (3)](images/pre_flight_checklist/qgc-parameters-3.png) |
|--------------------------|--------------------------|--------------------------|

- Download the default parameters to your drone.

![QGC Parameters (4)](images/pre_flight_checklist/qgc-parameters-4.png)

- Restart your drone.

Alternatively, you can use the [parameter template](parameters/crazyflie%202.1/) and configure the drone manually by installing [ArduPilot Methodic Configurator](https://github.com/ArduPilot/MethodicConfigurator) on your PC and following the on-screen instructions.

5. Calibrate Sensors (i.e. accelerometer, gyroscope, barometer).
- Plug your assembled drone into your computer with the micro USB cable.
- Open your GCS (Ground Control Station) of choice.
- Navigate to the sensor settings in your GCS.

![QGC Parameters (1)](images/pre_flight_checklist/qgc-parameters-1.png)

- Starting with the accelerometer, follow the on-screen instructions to calibrate each of the sensors referenced below:

![QGC Sensor Calibration](images/pre_flight_checklist/qgc-sensors.png)

- Restart the drone as directed.

6. Test Motors
- Plug your assembled drone into your computer with the micro USB cable.
- Open your GCS (Ground Control Station) of choice.
- Navigate to the motor settings in your GCS.
- Set the throttle to a low value (between 5 to 10%).

| ![QGC Parameters (1)](images/pre_flight_checklist/qgc-parameters-1.png) | ![QGC Motors](images/pre_flight_checklist/qgc-motors.png) |
|--------------------------|--------------------------|

- Test each motor individually, ensuring each motor spins in the correct direction as described in step 1 and each motor spins at approximately the same speed.

<video width="600" controls>
  <source src="images/pre_flight_checklist/motors-ind.mp4" type="video/mp4">
</video>

- Test all of the motors together and ensure all motors spin at approximately the same speed.

<video width="600" controls>
  <source src="images/pre_flight_checklist/motors-mixed.mp4" type="video/mp4">
</video>

## Telemetry Manual Mode
If you chose to flash the basic telemetry firmware to the AI Deck and you plan on flying drones manually, you will need to setup joystick control on your GCS before proceeding:

- [QGroundControl](https://docs.qgroundcontrol.com/Stable_V4.3/en/qgc-user-guide/setup_view/joystick.html)
- [Mission Planner](https://ardupilot.org/copter/docs/common-joystick.html)
- [MavProxy](https://ardupilot.org/mavproxy/docs/modules/joystick.html)

## Next Step, Flight!
You are now ready to begin flying ArduSwarm! Please proceed to the [First Flight Guide](first_flight.md).