# Drone's Software Development and Hardware Conception 

<img src="Prototype.jpg" border="10"/>

## A fully functional example project of a Quadcopter conception, including its environment which includes the Remote controller and a user interface (GUI)
This project details the design and creation of an autonomous drone, integrating advanced hardware and software components. The drone is powered by an STM32F4 microcontroller, acting as the control system's core. BLDC motors provide propulsion, and various sensors like the BMP280 for pressure, MPU6050 for acceleration, and a GPS module gather environmental and flight data. Communication is facilitated through an Xbee module. The software architecture includes modules for sensor data acquisition and processing, as well as control algorithms such as PI regulators and cascade techniques, ensuring the drone's stability and maneuverability. The practical implementation involved programming the microcontroller, integrating and calibrating sensors, precisely controlling the motors, and configuring the communication system.

## This repository contains 3 parts 

* The Flight controller folder, which is an STM32CubeIDE project to be uploaded to an STM32F401RE
* The Remote controller folder, which is Arduino Uno code
* A Python script that runs the GUI for useful information and also for automatic flight (to be added)
