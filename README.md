# robothon-ctrl-arm-del

This repository contains all of the source code for a **robotic arm** being developed for **Triton Droids' Robothon 2.0**. As part of the "twist" for this hackathon (humanoid robotics and targeting disability), we decided to give our robotic arm **6 degrees of freedom** and a design **inspired by human anatomy** in an attempt to better mimic the style and functionality of a real human arm.

## 🦾 Features
- 6 DOF to imitate level of mobility of a human arm
- More humanoid design than typical robot arms, weight should be more evenly distributed so it's not so base-heavy
- Controls over WiFi or Bluetooth using a phone or over a simple webserver
- (Stretch goal) Simple object/obstacle detection using ultrasonic sensors and/or camera

## 📁 Project Structure
At a glance, the project looks like this:
```
firmware/              # ESP32 firmware code
├── include/             # Project header files
├── lib/                 # External library files
├── src/                 # Source code (C++)
└── platformio.ini       # PlatformIO project info
server/                # Webserver or app code to contreol arm
└── ...                  # TBD
simulation/            # PyBullet simulation code
└── ...                  # TBD
```

## 🛠️ Tools Used
### Hardware List
- Freenove ESP32-WROVER (1)
- SG90 9g micro servo (6)
- MG996R 55g large servo (1)
  
(note: more to be added once things are finalized)

### Software List
- VS Code/PlatformIO (ESP32 firmware dev using C++ and Arduino)
- PyBullet (Arm model & simulations using Python)
- Fusion3D (Arm design & CAD modeling)
