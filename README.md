# Smart Automated Blinds

An IoT-enabled smart blind system that combines MQTT-based automation with traditional IR remote control. The system can learn and store custom IR remote codes, communicate wirelessly through MQTT, and automatically actuate motorized blinds based on subscribed scheduling events.

## Overview

This project was developed as part of an Internet of Things (IoT) course. The goal was to create a motorized blind controller capable of both manual and automated operation.

The system learns IR remote commands and stores user-defined mappings in EEPROM for persistent storage. Once configured, the controller can respond to IR remote inputs or automatically adjust the blinds through MQTT messages received from a connected broker.

- 🎥 [Demo Video](https://youtu.be/Oz5xw06XfSA)
   - **Note:** The blinds used in this demonstration were repurposed and had significant wear and corrosion, resulting in increased mechanical resistance and less smooth movement. The demonstration is intended to showcase the automation, networking, and control functionality rather than the mechanical performance of the blinds themselves.

## Features

- MQTT-based wireless blind control
- Automatic connection to an MQTT broker on startup
- Topic subscription for automated blind operation
- IR remote learning functionality
- EEPROM storage for persistent remote code mappings
- Motorized blind actuation
- User command interface for device configuration
- Dual control methods:
  - MQTT automation
  - IR remote control

## Technologies Used

- C/C++
- MQTT
- TCP/IP Networking
- EEPROM Memory
- Embedded Systems
- IR Communication
- Motor Control
- Internet of Things (IoT)

## System Architecture

The project consists of two primary components:

1. An MQTT-connected controller capable of learning and storing IR remote commands.
2. A motorized blind system that responds to MQTT messages and remote-control inputs.

After initialization, the device automatically connects to an MQTT broker, subscribes to configured topics, and listens for incoming commands. Users can also teach the system new remote-control codes, which are saved to EEPROM and retained after power cycles.

## Team Contributions

Both team members worked together on system integration, testing, and debugging throughout the project. We regularly reviewed each other's work, assisted in troubleshooting hardware and software issues, and collaborated to ensure reliable communication between the MQTT, IR, EEPROM, and motor control subsystems.

### Eric Nguyen
- Independently implemented and tested MQTT server communications during the initial networking phase.
- Led hardware development and system integration.
- Designed and assembled the mechanical blind actuation system.
- Performed all soldering and hardware wiring.
- Developed the user interface and command system used to configure remote-control mappings.
- Implemented EEPROM functionality for persistent storage of learned IR commands.

### Kaven
- Independently implemented and tested MQTT server communications during the initial networking phase.
- Developed MQTT functionality for the final system.
- Implemented automatic broker connection and topic subscription at startup.
- Created the IR receiver module and learning state machine.
- Developed functionality for recognizing and storing custom IR remote codes.

## What We Learned

Through this project we gained experience with:

- Embedded systems development
- MQTT publish/subscribe communication
- TCP/IP networking
- EEPROM data storage
- IR signal processing
- Motor control systems
- Hardware assembly and soldering
- Collaborative software and hardware development
- IoT system integration

## Future Improvements

- Mobile application interface
- Web-based configuration dashboard
- Voice assistant integration
- Blind position feedback sensors
- Cloud-based scheduling and monitoring
- Enhanced security and authentication for MQTT communication
