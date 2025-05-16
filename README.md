# robothon-ctrl-arm-del

This repository contains all of the source code for a **robotic arm** being developed for **Triton Droids' Robothon 2.0**, which ran over the course of a few weeks (and actual time given for development was only a couple of weeks). As part of the "twist" for this hackathon (humanoid robotics and targeting disability), we decided to give our robotic arm **6 degrees of freedom** and a design **inspired by human anatomy** in an attempt to better mimic the style and functionality of a real human arm.

## 🦾 Features
- 6 DOF to imitate level of mobility of a human arm
- More humanoid design than typical robot arms, with an improved weight distribution
- Controls over WiFi in a web browser using a WebSocket server

## 📷 The Final Product
<p align="center">
  <img src="/public/arm.png" width="80%"></img>
</p>

## 🛠️ Tech
### Hardware Used
- Freenove ESP32-WROVER (1)
- SG90 9g micro servo (5)
- MG996R 55g large servo (1)
- 5V/6A power supply
  - Any method of supplying this amount of power works, but we used a USB-C PD charger and 2 of [these](https://a.co/d/63WjR2B) trigger modules to step down 20V in to 5V out.
- Breadboard (1)
- Jumper wires (at least 20; 18 for 6 servos, 2 for the ESP)

### Software Used
- VS Code/PlatformIO (ESP32 firmware dev using C++ & Arduino)
- PyBullet (Arm model & simulations using Python)
- Fusion3D (Arm design & CAD modeling)

## 📁 Project Structure
At a glance, the project looks like this:
```
firmware/              # ESP32 firmware code
├── include/             # Project header files
├──── credentials.h         # WiFi SSID & password
├── lib/                 # External library files
├── src/                 # Source code (C++)
├──── main.cpp              # Main firmware definitions & loop
├──── server.cpp            # WebSocket server for controlling servos
└── platformio.ini       # PlatformIO project info
public/                # Assets used for README and such
└── ...
simulation/            # PyBullet simulation code
└── ...
```

## 👤 Team
This project was made possible only by the combined efforts of the CTRL-ARM-DEL team:
- Steven: Project coordination, research, and hardware
- Brandon: CAD modeling and hardware
- Kai: CAD modeling and hardware
- Daniel: Hardware, electronics, and simulation
- Miles: Electronics, wiring, and firmware
