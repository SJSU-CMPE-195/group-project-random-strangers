# Project Title

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

## Proof of Concept Scope

Demonstrate a design that includes:

- [ ] **Fully working mechanical system**
    - [ ] Gimbal with control in both yaw and roll orientations
    - [ ] Flywheel-based dodgeball launching system

- [ ] **Control system for setting motor parameters**
    - [ ] Webserver interaction
    - [ ] I2C-based configuration of ESC drivers
    - [ ] Parameters persist across resets

- [ ] **Targeting and firing system**
    - [ ] Webserver integration to select point on screen
    - [ ] Trajectory estimation

---

# Implementation  

## Screenshots  
| Feature | Screenshot |
|---------|------------|

## Tech Stack

| Description | Category | Technology | Running On |
|----------|------------|-------------|------------|
| Command and control webserver | Frontend | Next.JS | Jetson |
| Depth Sensing | Sensing | Microsoft Kinect 2 with libfreenect2 | Jetson |
| High Speed BLDC Control | Movement | SimpleFOC | ESCs |

## Getting Started  
 **TODO** Add a tutorial to getting all the parts working :)  

---

## License

This project is licensed under the [LGPLv3 License](www.gnu.org/licenses/lgpl-3.0.en.htm) - see the [LICENSE](LICENSE.md) file for details.

---

*CMPE 195A/B - Senior Design Project | San Jose State University | Spring 2026*
