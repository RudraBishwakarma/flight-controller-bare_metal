# Software Design

## Overview

The Bare-Metal Flight Controller follows a modular and layered software
architecture designed for deterministic execution, maintainability, and
extensibility. Each module performs a single engineering responsibility and
communicates through well-defined interfaces, reducing coupling between
subsystems.

Unlike an RTOS-based implementation, the firmware uses a cooperative scheduler
to execute periodic tasks at fixed frequencies. This approach minimizes runtime
overhead while ensuring predictable timing for flight-critical operations.

---

# Design Goals

The software architecture was designed with the following objectives:

- Deterministic real-time execution
- Modular software organization
- Hardware abstraction
- Ease of debugging and testing
- Scalability for future features
- Separation of hardware and application logic
- Safety-first operation

---

# Software Architecture

```
                    Application
                         │
        ┌────────────────┼────────────────┐
        ▼                ▼                ▼
   Flight Control   Communication     Safety
        │                │                │
        └────────────────┼────────────────┘
                         ▼
                    Estimation
                         │
                         ▼
                     Device Drivers
                         │
                         ▼
             Hardware Abstraction Layer
                         │
                         ▼
                 STM32F4 Hardware
```

Each layer depends only on the layer directly below it.

---

# Module Organization

```
flight-controller/

├── application/
├── control/
├── estimation/
├── communication/
├── safety/
├── scheduler/
├── drivers/
├── hal/
├── simulation/
├── tests/
├── tools/
├── docs/
└── Makefile
```

---

# Software Modules

## Application Layer

Coordinates the complete firmware.

Responsibilities

- System initialization
- Module startup
- Task registration
- Scheduler startup
- Flight state management

---

## Scheduler

The scheduler periodically executes every firmware task.

Responsibilities

- Task scheduling
- Time management
- Frequency control
- Cooperative execution

Typical frequencies

| Task | Frequency |
|-------|-----------|
| IMU Sampling | 1000 Hz |
| Sensor Fusion | 1000 Hz |
| PID Control | 500 Hz |
| SBUS Processing | 100 Hz |
| Telemetry | 20 Hz |
| Failsafe | 100 Hz |

---

## Drivers

Provide low-level access to external hardware devices.

Current drivers include

- MPU6000 IMU
- SPI
- UART
- PWM
- Timers

Drivers expose simple APIs while hiding hardware-specific implementation.

---

## Estimation

Converts raw sensor measurements into aircraft orientation.

Components

- IMU Calibration
- Mahony Filter
- Attitude Estimator

Output

- Roll
- Pitch
- Yaw

---

## Flight Control

Stabilizes the aircraft using cascaded PID control.

Components

- Angle Controller
- Rate Controller
- PID Library
- Motor Mixer

Output

Individual motor commands.

---

## Communication

Handles interaction with external devices.

Components

- SBUS Receiver
- MSP Protocol
- Telemetry

Responsibilities

- Receive pilot commands
- Decode packets
- Transmit telemetry
- Ground station communication

---

## Safety

Monitors system health and protects the aircraft.

Components

- RC Failsafe
- Arming Logic
- Watchdog

Safety logic operates independently of the controller.

---

## Hardware Abstraction Layer

Provides hardware-independent interfaces.

Interfaces include

- GPIO
- SPI
- UART
- Timers
- PWM

Higher software layers never directly access STM32 registers.

---

# Data Flow

The firmware continuously transforms pilot commands into motor outputs.

```
SBUS Receiver

↓

Command Processing

↓

Angle Controller

↓

Rate Controller

↓

Motor Mixer

↓

PWM Output

↓

ESC

↓

Motors
```

Meanwhile

```
MPU6000

↓

Calibration

↓

Mahony Filter

↓

Attitude Estimate

↓

PID Controller
```

---

# Initialization Sequence

At power-up the firmware initializes modules in dependency order.

```
Reset

↓

Clock Configuration

↓

HAL Initialization

↓

Peripheral Initialization

↓

Driver Initialization

↓

Sensor Calibration

↓

PID Initialization

↓

Scheduler Initialization

↓

Start Control Loop
```

---

# Task Execution Model

Every task follows the same lifecycle.

```
Task Ready

↓

Read Inputs

↓

Process Data

↓

Generate Outputs

↓

Return
```

Tasks never block execution, ensuring all scheduled activities complete within
their assigned periods.

---

# Communication Between Modules

Modules interact only through public interfaces.

Example

```
Control

↓

Estimation API

↓

Mahony Filter

↓

IMU Driver

↓

SPI HAL
```

This prevents higher-level modules from depending on implementation details.

---

# Error Handling

Each module validates its inputs and reports errors through standardized status
codes.

Examples

- Invalid sensor data
- Communication timeout
- Receiver loss
- Initialization failure

Critical faults invoke the safety subsystem to place the aircraft in a safe
state.

---

# Memory Organization

The firmware uses statically allocated memory wherever possible.

Memory regions include

- Program Flash
- Global variables
- Task state
- Communication buffers
- Sensor data
- Stack

Dynamic memory allocation is intentionally avoided to eliminate fragmentation
and improve runtime predictability.

---

# Design Principles

### Modular Design

Each module has a single engineering responsibility.

---

### Layered Architecture

Higher-level software depends only on lower-level interfaces.

---

### Hardware Independence

Application logic remains portable by accessing peripherals through the HAL.

---

### Deterministic Execution

Fixed update rates ensure consistent control-loop timing.

---

### Testability

Modules can be validated independently using simulation and unit tests.

---

### Extensibility

Future features such as GPS, barometer, magnetometer, MAVLink, or autonomous
navigation can be integrated by adding new modules without redesigning the
existing architecture.

---

# Advantages of the Design

- Predictable real-time behaviour
- Low runtime overhead
- Simple debugging
- High code readability
- Easy maintenance
- Reusable modules
- Scalable architecture
- Clear separation of concerns

---

# Future Improvements

Potential extensions include:

- Extended Kalman Filter (EKF)
- GPS navigation
- Barometer support
- Magnetometer integration
- Optical flow estimation
- MAVLink protocol
- Mission planning
- Autonomous flight modes
- Data logging to SD card
- RTOS-based scheduling for advanced multicore or high-complexity systems

---

# Summary

The software design emphasizes simplicity, modularity, and deterministic
execution. By separating estimation, control, communication, safety, and
hardware abstraction into independent layers, the firmware remains easy to
understand, test, and extend.

This architecture reflects software engineering practices commonly used in
professional embedded systems and provides a strong foundation for developing
reliable flight control firmware on resource-constrained microcontrollers.
