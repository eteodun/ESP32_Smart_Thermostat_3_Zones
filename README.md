#  Module ESP designed to be used for Smart Home heating system with 3 zones integrated with Home Assistant via MQTT protocol.

# ESP Module for Home Assistant-Integrated Heating System

This project is an open-source solution that integrates an ESP module with Home Assistant to control a heating system. The system consists of a boiler for centralized heating and three individual heating zones. Each zone is managed by a TRV (Thermostatic Radiator Valve) that operates via Zigbee technology, ensuring precise temperature control in each area. Communication between the ESP module and Home Assistant is facilitated through MQTT, providing real-time data exchange and control.

## Key Features:

Seamless Home Assistant Integration: Easily control and monitor the heating system through Home Assistant.

Zonal Control: Manage three independent heating zones with TRV valves over Zigbee for efficient and customized heating.

Real-Time MQTT Communication: The ESP module communicates with Home Assistant via MQTT, ensuring swift updates and commands.

Customizability: Adjust heating schedules, temperature setpoints, and other parameters to meet specific needs.

Protection to over running boiler: The system has implemented conditions to prevent boiler over running  in case that one or more sensor fail to measure, in case that  wifi or Home Asssitant is disconnected or malfunction on heating instalation that can increase time to reach or not reach target temperature.

## Technologies Involved:

ESP Microcontroller (ESP8266/ESP32): The core component that runs the control logic and manages communications.
Home Assistant: The home automation platform that integrates all components of the heating system.
MQTT Protocol: A lightweight messaging protocol used for fast and efficient communication between the ESP module and Home Assistant.
Zigbee: The wireless communication standard used to control the TRV valves.
#How It Works:

Upon startup, the ESP module connects to the Wi-Fi network and establishes an MQTT connection with Home Assistant. The module then listens for parameters from Home Assistant to adjust temperature setpoints and schedules for each heating zone. When a command is received, it transmits the necessary instructions over Zigbee to control the corresponding TRV valve. This setup allows Home Assistant to continuously monitor system performance and make real-time adjustments, ensuring optimal comfort and energy efficiency.

Project Setup Overview:

To implement the system, you need to configure the ESP module to connect to your Wi-Fi and MQTT broker, set up the Zigbee communication for the TRV valves, and integrate the device within Home Assistant. Once connected, Home Assistant will manage the heating zones based on real-time data and user-defined settings.

Contributing:

Contributions to improve the project are welcome. If you find any issues or have suggestions for enhancements, feel free to share your ideas or submit pull requests. Collaborative efforts will help refine the system and expand its capabilities.

License:

This project is released under an open-source license, allowing for modifications and redistribution. For detailed licensing information, please refer to the LICENSE file in the repository.

This comprehensive solution aims to provide a reliable and flexible approach to managing home heating, combining modern communication protocols with effective hardware integration for an enhanced user experience.   
