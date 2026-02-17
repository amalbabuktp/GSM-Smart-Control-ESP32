# GSM Smart Motor Control and Monitoring System

## Overview
This project implements an SMS-based remote motor control system using ESP32.  
The system allows remote ON/OFF control, overload detection, vibration monitoring, and timer-based protection.

## Features
- SMS-based motor control (ON / OFF)
- Overload protection using ACS712
- Vibration detection using SW-420
- Real-time clock scheduling (DS3231)
- Temperature and humidity monitoring (DHT11)
- Automatic safety shutdown

## Hardware Components
- ESP32
- SIM800L GSM module
- ACS712 Current Sensor
- DS3231 RTC
- DHT11
- SW-420 Vibration Sensor
- Relay Module

## Folder Structure
- `code/` → ESP32 firmware
- `hardware/` → Circuit diagram
- `images/` → Project setup images

## Working Principle
The system listens for SMS commands via SIM800L.  
Upon receiving valid commands, the ESP32 controls the motor relay.  
Sensor data is continuously monitored for overload and vibration faults.  
If abnormal conditions are detected, the motor is automatically shut down and SMS alerts are sent.

## Author
Amal Babu K

