# ArduSwarm: A Robust Test Platform for Swarm Robotics Research
## Overview
**ArduSwarm** is a project developed as part of a Master's in Mechanical Engineering, designed to provide a robust and scalable test platform for research into drone swarm dynamics and defensive algorithms. The platform leverages the **Bitcraze Crazyflie 2.1** drone, a small yet highly capable quadcopter, and integrates it with the industry-standard **ArduPilot** flight control software (ArduCopter).

The primary goal of this project is to create a research tool that enables the practical testing of security algorithms within a distributed drone swarm. This involves significant development to port the Crazyflie hardware onto the ArduPilot software, unlocking a wide feature set for advanced research applications.

## 🚀 The Platform
### Hardware: Bitcraze Crazyflie 2.1
The **Crazyflie** platform was chosen for its unique combination of a small form factor, powerful capabilities, and a fully open-source ecosystem. It is targeted at researchers and enthusiasts, offering features crucial for swarm robotics, including:

- **Peer-to-peer communication**
- **Onboard localization** for GPS-denied environments
- Support for expansion decks, such as the **Flow Deck** for optical flow and ranging, and the **AI Deck** for onboard computation.

### Software: ArduPilot (ArduCopter)
**ArduPilot** is a professional-grade, open-source flight stack. Its adaptability and extensive feature set make it an ideal choice for this research platform. Key features include:

- A robust **L1 controller**
- **Lua scripting** for easy customization of drone behavior
- Support for various navigation methods, including **non-GPS localization**

A significant portion of this project has been dedicated to overcoming the limited default support for the Crazyflie in ArduPilot, enabling a powerful combination of hardware and software.

## ✨ Key Features
This platform is designed to meet the demands of advanced swarm robotics research, with a focus on the following requirements:

- **Peer-to-Peer Communication**: Enables drones to relay state updates and commands directly to each other.
- **Non-GPS Localization**: Utilizes the Flow Deck with its optical flow and Time-of-Flight (ToF) sensors for indoor navigation.
- **Relative Positioning**: Allows drones to be aware of their peers' positions for collision avoidance and formation control.
- **Robustness & Scalability**: Designed to perform reliably in various conditions and to scale from a few drones to dozens.
- **Ease of Use**: Aims to provide a straightforward user experience for all members of the research group, regardless of their hardware or software expertise.
- **Open Source**: Both the hardware and software are fully open-source, promoting collaboration.
- **Onboard Computation**: Capable of running higher-order algorithms directly on the drone.

## 🛠️ Getting Started
This section provides a guide to setting up a Crazyflie drone with the custom ArduPilot firmware developed for this project.

1. **Compiling and Flashing the Firmware**
The first step is to compile the custom ArduPilot firmware and flash it onto the Crazyflie. This process requires setting up a build environment for ArduPilot on your operating system.

For detailed instructions, please refer to the [Compiling & Flashing Guide](compiling_and_flashing.md).

2. **Enabling Onboard Sensors**
For indoor navigation, you will need to enable the optical flow and rangefinder sensors on the Flow Deck.

**Optical Flow**: For detailed instructions on enabling the PWM3901 optical flow sensor, see the [Optical Flow Guide](optical_flow.md).

**Rangefinder (ToF)**: To enable the VL53L1x Time-of-Flight sensor, follow the [RangeFinder Guide](rangefinder.md).

3. **Using Lua Scripting for Custom Behavior**
Lua scripting allows you to add custom logic to the drone's behavior without modifying the core C++ flight code. This is ideal for implementing and testing new algorithms.

To get started with Lua scripting on the Crazyflie, please see the [Lua Scripting Guide](lua_scripting.md).

## 💻 Development Notes
### Memory Optimization
The STM32 MCU on the Crazyflie has limited flash memory (1 MB). As you add features to your custom firmware, you may exceed this limit.

If you encounter a build failure, you will likely need to free up memory by disabling certain ArduPilot features. For a detailed guide on how to do this, please refer to the [Freeing up Memory Guide](freeing_up_memory.md).

### Restoring the Crazyflie to Factory Firmware
If you need to revert the Crazyflie to its original Bitcraze firmware for any reason, you can follow a straightforward restoration process.

For step-by-step instructions, please see the [Restoring the Crazyflie Guide](restoring_the_crazyflie.md).

## 🔮 Future Work
This platform provides a foundation for a wide range of swarm robotics research. Future work will include:

- Implementing and testing specific differential game-based defensive algorithms.
- Integrating the AI Deck for more intensive onboard computation.
- Developing a user-friendly interface for managing swarm experiments.
- Porting the NRF51 MCU for onboard radio communication.
- Implementing a peer-to-peer communication protocol for larger swarms.

## ✍️ Author
**Kwaku Richter**, Masters Student in Mechanical Engineering at the University of Ottawa

📧: frich089@uottawa.ca