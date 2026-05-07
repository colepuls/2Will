# Self-Balancing Robot - 2Will

A two-wheeled self-balancing robot built with an ESP32, BNO055 IMU, custom PCB, servo-controlled legs, and Xbox controller input. The robot uses a PID-based balance loop to stay upright while supporting standing, crouching, small forward/backward movements, and 90-degree turns.

## Project Overview

This project was designed to explore embedded control systems, PCB design, mechanical CAD, and real-time robot balancing. The robot reads pitch data from a BNO055 IMU and uses PID control to drive two DC motors, keeping the robot balanced on two wheels.

The robot also includes servo-controlled legs that allow it to switch between standing and crouching positions. An Xbox controller is used to control the robot’s position, movement, and turning features.

## Images

### SCHEMATIC

![PCB](images/schematic.png)

### PCB

![CAD](images/pcb.png)

### CAD

![CAD](images/cad.png)

## DEMO

[link](https://www.youtube.com/shorts/AqlbXCcy0CI)

## Features

- Self-balancing using PID control
- ESP32-based control system
- BNO055 IMU for pitch and yaw readings
- Custom PCB for cleaner wiring and hardware integration
- Servo-controlled standing and crouching positions
- Xbox controller support through Bluepad32
- D-pad left/right 90-degree turning
- D-pad up/down small forward and backward movement
- Fall cutoff safety to stop motors when the robot tips too far

## Hardware

- ESP32 development board
- BNO055 IMU
- Dual DC motors
- Motor driver - TB6612FNG
- 20kg Servo motors
- Custom PCB
- Battery power supply
- 3D printed
- Xbox controller

## Software / Libraries

- Arduino framework
- `Wire.h`
- `Adafruit_BNO055`
- `ESP32Servo`
- `Bluepad32`

## Controls

| Button | Action |
|---|---|
| A | Stand |
| B | Crouch |
| X | Toggle left leg |
| Y | Toggle right leg |
| D-pad Left | Turn left 90 degrees |
| D-pad Right | Turn right 90 degrees |
| D-pad Up | Move forward slightly |
| D-pad Down | Move backward slightly |

## How It Works

The robot constantly reads its pitch angle from the BNO055 IMU. The current pitch is compared to a target pitch, and the PID loop calculates how much motor power is needed to keep the robot balanced.

The robot uses different PID values for standing and crouching. While standing, the proportional gain changes depending on how far the robot is leaning. This helps keep the robot smoother near the balance point while still giving it enough power to recover from larger pushes.

For turning, the robot uses yaw data from the IMU. One motor receives slightly more output while the other receives less, allowing the robot to rotate while still balancing.

Forward and backward movement is handled by briefly shifting the target pitch. This lets the robot move a small amount while still using the balance loop.

## Current Status

The robot is built and balancing. The main focus right now is tuning the PID values, improving recovery after pushes, and refining movement controls.

## To Do

- Improve push recovery with an actual outer velocity loop
- Add cleaner calibration on startup
- Tune movement and turning for smoother control