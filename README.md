# Automatic Dodgeball Turret

> Automatic Dodgeball Turret for Defence and ATTACK!

## Team

| Name | GitHub | Email |
|------|--------|-------|
| Zoya Shamak | [@ZoyaS](https://github.com/ZoyaS) | zoya.shamak@sjsu.edu |
| Isa Pudiyapura | [@github210878](https://github.com/github210878) | isa.pudiyapura@sjsu.edu |
| Eric Huang | [@eiechuang](https://github.com/eiechuang) | eric.huang01@sjsu.edu |
| Brendan Parvin | [@brendanofawesome](https://github.com/Brendanofawesome) | brendan.parvin@sjsu.edu |

**Advisor:** [Prof. Wencen Wu]

---

# Project Design  

## Problem Statement  

**Dodgeball is a timeless game of skill and agility. We hope to fix that!**  
With our turret, no longer do you need any particular ability in order to:
 - aim and fire dodgeballs at targets
 - automatically defend against incoming dodgeballs using *hit-to-kill* 

# Proof-of-Concept Scope

Demonstrate a functional prototype capable of detecting, tracking, aiming at, and launching an interceptor toward a dodgeball target.

## Status Legend

- ✅ Complete
- 🚧 In progress
- ⬜ Not started

## 1. Mechanical System

- 🚧 **Fully functional mechanical system**
  - 🚧 Gimbal with control in both yaw and pitch axes
    - ✅ Virtual model
    - 🚧 Physical model
    - ⬜ Testing
  - 🚧 Flywheel-based dodgeball launching system
    - ✅ Flywheel design
    - ⬜ Launch-trigger design
    - ⬜ Testing

## 2. Electrical System

- 🚧 **Fully functional electrical system**
  - ✅ Yaw motor operating with position feedback
  - ✅ Pitch motor operating with position feedback
  - 🚧 Slip ring installed on the yaw axis
  - 🚧 Flywheel ESCs operating with feedback
  - 🚧 Motor power supply operational
  - 🚧 JST-style electrical connections implemented

## 3. Motor-Control System

- 🚧 **System for configuring and controlling motor parameters**
  - 🚧 I2C Drivers for Jetson
  - 🚧 I2C Drivers for Launch Controller
  - ⬜ I2C Driver for Gimbal Controllers
  - ⬜ Motor parameters persist across resets
  - ⬜ Web-server control interface

## 4. Targeting and Firing System

- 🚧 **Targeting and Firing system**
  - ⬜ Camera-to-gimbal coordinate transformation
  - ⬜ Interceptor Trajectory Estimation
  - ⬜ Web-server control for firing at an arbitrary point on the camera view
  - ⬜ Intercept-time and position calculator
  - 🚧 Target tracking and trajectory estimatin
  - ⬜ Firing safety interlocks
  - ⬜ Launch confirmation and failure detection

## 5. System Integration

- ⬜ End-to-end operation from target detection to firing
- ⬜ Communication between the Jetson and all motor controllers
- ⬜ Emergency-stop functionality
- ⬜ Startup, shutdown, and fault-recovery behavior
- ⬜ Repeatable firing accuracy testing
- ⬜ System performance and latency measurements
    

---

# Implementation  

## Screenshots  
| Feature | Screenshot |
|---------|------------|

## Tech Stack

| Function | Category | Technology | Runs On |
|---|---|---|---|
| Command and control interface | Frontend | Next.js | Jetson |
| System command processing | Backend | fastAPI Python service | Jetson |
| Target interception planning | Planning | Intercept Optimizer in Python | Jetson |
| Trajectory estimation | Planning | Linear Regression in Python | Jetson |
| Target recognition | Sensing | YOLOv26 | Jetson |
| Depth measurement and Tracking | Sensing | ZED 2i Stereo Camera and ZED SDK | Jetson |
| Motor-controller communication | Communication | Custom I2C protocol | Jetson and Microcontrollers |
| Gimbal position control | Low-level control | Hall-Effect Angle Sensing and SimpleFOC | Pitch Motor Controllers (2×ESP32) |
| Flywheel speed control | Movement | ESC Feedback Control | Flywheel Controller (ESP32) |

## Getting Started  
 **TODO** Add a tutorial to getting all the parts working :)  

---

## License

This project is licensed under the [LGPLv3 License](www.gnu.org/licenses/lgpl-3.0.en.htm) - see the [LICENSE](LICENSE.md) file for details.

---

*CMPE 195A/B - Senior Design Project | San Jose State University | Spring 2026*
