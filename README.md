# robothon-ctrl-arm-del

This repository will contain the source-code for an ESP32-powered humanoid robotic arm, as our team's submission to the Triton Droids Robothon 2.0 (running 'till May 17). I'm keeping it public just because, but until this README says otherwise, any code in here is NOT finished and will likely not work correctly :P

## Planned features
- 6 DOF to imitate level of mobility of a human arm
- More humanoid design than typical robot arms, weight should be more evenly distributed so it's not so base-heavy
- Controls over WiFi or Bluetooth using a phone
- (Stretch goal) Simple object/obstacle detection using ultrasonic sensors and/or camera

## Planned hardware list
- 1x ESP32-S3
- 6x NEMA 17 servo motors
- 1x IMU
- Additional motors as needed to support weight/increase weight capacity
- 1x some sort of external power supply to power all of these motors lol

## Planned software list
- PlatformIO for ESP32 development
- PyBullet for running simulations
- Fusion3D for CAD modeling
