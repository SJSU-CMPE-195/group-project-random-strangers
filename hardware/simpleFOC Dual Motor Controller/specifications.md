# SimpleFOC Dual Motor Controller - Specifications

## Overview
High-performance dual BLDC motor driver with FOC (Field-Oriented Control) algorithm support, designed for applications requiring precise motor control with simultaneous dual-motor operation.

---

## Power Specifications

### Input Power
- **Voltage Range**: 0V - 36V (nominal 36V maximum)
- **Continuous Power**: As determined by thermal dissipation and current limits (IDK)
- **Voltage Input Protection**: 
  - [x] Overvoltage protection (TVS diodes or Zener clamps)
  - [ ] Reverse polarity protection (ideal diode or integrated FET)
  - [ ] Surge suppression for inductive loads

### Current Specifications
- **Continuous Current per Motor**: 35A (per motor pole)
- **Peak Continuous Current**: 64A (per motor pole)
- **Current Sense Method**: integrated low side current sensing per pole
- **Current Limiting**: Software-controlled with adjustable thresholds

### Power Dissipation
- **MOSFET Losses**: 35A * 4mΩ * 2 = 0.28W per pole
- **Thermal Management**: Heat sinks required for continuous high-current operation
- **Thermal Shutdown**: Integrated temperature monitoring with cutoff at 80-85°C

---

## Motor Control Features

### Supported Motor Types
- **Primary**: 3-phase BLDC (Brushless DC) motors
- **Motor Poles**: 2 to 12 pole pairs typical range
- **Motor Power Range**: 0W to 3kW (voltage and current dependent)

### Encoder/Sensor Support
- **Hall Effect Sensors**:
  - Support for all Hall configurations (commutation angles)
  - Automatic Hall state detection and phase alignment
  
- **Encoder Interface**:
  - Dedicated input pins per motor for Hall sensor signals
  - Hardware debouncing (optional RC filters)
  - Software debounce filtering with configurable delay

### Control Algorithms
- **FOC (Field-Oriented Control)**: Primary control mode
- **Commutation**: Hall-based 6-step or smooth vector control
- **Velocity Control**: PID-based speed regulation
- **Current Control**: Inner PID loop for precise current management
- **Position Control**: Incremental position tracking

---

## Hardware Architecture

### Motor Driving Stage (Per Motor)
- **Topology**: 3-phase full-bridge (6-MOSFET inverter)
- **Gate Driver**: discrete high/low-side gate drivers
  - Deadtime compensation to prevent cross-conduction
  - Bootstrap capacitor charging for high-side drivers
  - Shoot-through protection
  
- **Power Semiconductors**:
  - N-channel MOSFETs with low RDS(on) (<4mΩ typical at 25°C)
  - Voltage rating: ≥60V to handle transient spikes
  - Current rating: ≥50A continuous (per phase)

### Current Sensing (Per Motor)
- **Method**: Low-side shunt resistor sensing
- **Shunt Resistance**: Typical 0.001Ω - 0.01Ω (precision resistor ±1%)
- **Current Signal Processing**:
  - Op-amp based amplification and filtering
  - Signal level conversion to ADC-compatible range (0-3.3V)
  - DC offset rejection and noise filtering

### Hall Sensor Interface
- **Input Impedance**: High impedance (pull-up to 3.3V or 5V)
- **Debouncing**: Hardware (RC filter, typically 1-10µs time constant)
- **Signal Conditioning**: Schmitt trigger inputs for noise immunity
- **Per-Motor Configuration**: Independent hall input pins for dual-motor operation

### Microcontroller/Controller Board
- **Integrated Microcontroller**: 
  - **STM32G4 Series**: Cost-effective FOC control with sufficient processing power for dual motor management

### Power Distribution
- **Input Filtering**:
  - Bulk capacitance: 220µF @ 100V
  - High-frequency ceramic capacitors: 10uF x 12
  - LC filtering for noise reduction
  
- **Voltage Rails**:
  - Main power bus: +36V (directly from input)
  - Logic supply: +5V
  - Gate driver supply: 10V Charge Pump

---

## Electrical Specifications

### Output Specifications (Per Motor)
- **Output Voltage**: 0V - VIN (PWM modulated)
- **Frequency**: 16-20 kHz typical (configurable)
- **Phase Current Range**: 0A - 35A (continuous)

### Protection Features
- **Over-Current Protection**: Software-programmable limits per motor
- **Over-Temperature**: Thermal shutdown with hysteresis
- **Phase Fault Detection**: Open-phase or short-circuit detection
- **Under-voltage Lockout**: Prevents operation below safe voltage threshold
- **Over-voltage Clamping**: TVS diodes on phase outputs

### Control Signals
- **PWM Frequency**: 16-20 kHz (20-25 kHz for low audible noise)
- **Deadtime**: 50-200ns (tunable to minimize losses and cross-conduction)
- **Switching Resolution**: 12-bit typical (configurable)

---

## Communication Interface

### I2C Slave Interface
- **Protocol**: I2C slave
- **Clock Speed**: Up to 800 kHz (configurable)
- **Data Width**: 8-bit transfers
- **Features**:
  - Real-time motor control commands
  - Telemetry readback (speed, current, temperature)
  - Parameter configuration and tuning
  - Fault/diagnostic reporting and clearing
  - Low latency (<1ms round-trip response)

### Backup Communication
- **USB Debug Port**: 115200 baud for development/debugging only
- **Protocol**: UART for system diagnostics

### Configuration
- **Parameter Storage**: Flash memory for persistent settings
- **Firmware Updates**: Via USB interface with checksumming
- **Startup Calibration**: Automated hall sensor calibration on power-up or via SPI command

---

## Performance Specifications

### Control Performance
- **Current Control Loop Frequency**: 10-20 kHz
- **Velocity Control Loop Frequency**: 1-5 kHz (configurable)
- **Current Ripple**: <5% at rated current
- **Efficiency**: >90% at rated current (at 36V input)

### Sensor Performance
- **Hall Sensor Polling Rate**: 20 kHz minimum
- **Hall-to-PWM Update Latency**: <100µs
- **Motor Startup Time**: <500ms from initialization to controlled rotation

---

## Thermal Management

### Temperature Monitoring
- **Sensor Location**: On heatsink or MOSFET thermal pad
- **Temperature Range (Operating)**: -10°C to +85°C
- **Temperature Range (Storage)**: -20°C to +100°C
- **Thermal Shutdown**: 80-85°C (with 5-10°C hysteresis)

### Cooling Requirements
- **Passive Cooling**: Adequate heatsinks for 35A continuous operation
- **Active Cooling**: Optional fan control for high-duty-cycle applications
- **Thermal Design**: Capable of dissipating >600W continuously

---

## Physical and Mechanical

### PCB Design
- **Layer Count**: 4-layer minimum (power, ground planes essential)
- **Trace Width**: Calculated for 35A+ with <10mV drop per phase
- **Thermal Vias**: Liberally used under MOSFET pads and power components
- **Isolation**: 3kV reinforced isolation between control and power sections (optional)

### Connector Standards
- **Power Input**: High-current connector (XT60, XT90, or Anderson PowerPole)
- **Motor Output**: 3.5mm or 5mm phoenix connectors per motor
- **Hall Sensors**: JST-XH or similar low-current connectors (6 pins per motor)
- **Communication**: USB micro-B or UART header

### Dimensions
- **Target Size**: ~150mm × 100mm × 50mm (estimated, depends on heatsink size)
- **Weight**: <500g (without heatsink)

---

## Software Architecture

### Firmware Platform
- **Framework**: simpleFOC library v2.3+ with dual-motor support
- **Development Language**: C/C++

### Core Control Loop
- **Execution**: Hardware timer interrupt (20 kHz)
- **Tasks**:
  1. Read hall sensor states
  2. Estimate rotor angle via sensor fusion
  3. Calculate FOC voltage vectors
  4. Update PWM outputs
  5. Log telemetry (if enabled)

### Tuning Parameters
- **PID Gains** (per motor):
  - Current loop: Kp, Ki (typically Kp=10-50, Ki=100-500)
  - Velocity loop: Kp, Ki, Kd
- **Motor Parameters**: Pole pairs, voltage rating, current rating
- **Hall Configuration**: Specific angle mapping for hall state transitions

---

## Compliance and Safety

### Electrical Safety
- **Isolation**: Optional galvanic isolation between control and power circuits
- **Fusing**: Input fuse rated for 40-50A @ 48V (protection from short-circuit)
- **Creepage/Clearance**: PCB design follows IEC standards

### Functional Safety
- **Watchdog Timer**: Monitors firmware execution and resets on timeout
- **Redundant Checks**: Current and temperature limits with independent enforcement
- **Fault Reporting**: Detailed error codes for diagnostics and debugging

---

## Typical Applications

- Dual-axis robotic actuators
- Autonomous vehicle drive systems
- High-performance drone motors
- Gimbal stabilization systems
- Industrial automation drives
