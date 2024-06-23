# Drone's Software development and Hardware conception 

<img src= "Prototype.jpg" border="10"/>

## A fully functional example project of a Quadcopter conception including it's envirement wich include the Remote controller and a user interface (GUI)
This project details the design and creation of an autonomous drone, integrating advanced hardware and software components.
The drone is powered by an STM32F4 microcontroller, acting as the control system's core. BLDC motors provide propulsion,
and various sensors like the BMP280 for pressure, MPU6050 for acceleration, and a GPS module gather environmental and flight data. Communication is facilitated through an Xbee module.
The software architecture includes modules for sensor data acquisition and processing, as well as control algorithms such as PI regulators and cascade techniques,
ensuring the drone's stability and maneuverability. The practical implementation involved programming the microcontroller,
integrating and calibrating sensors, precisely controlling the motors, and configuring the communication system.
