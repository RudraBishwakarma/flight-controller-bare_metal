# Flight Controller Architecture

## Overview

The flight controller is organized as a layered embedded software system that
separates hardware access, estimation, control, communication, and safety into
independent modules.

Each subsystem performs one engineering responsibility while communicating
through well-defined interfaces. This modular organization minimizes coupling,
simplifies debugging, and allows new functionality to be integrated without
major architectural changes.

The firmware executes on an STM32F4-class microcontroller without an operating
system, using a deterministic cooperative scheduler.

---

# High-Level Architecture

```
                 Pilot Commands
                       │
                       ▼
                SBUS Receiver Driver
                       │
                       ▼
             Flight Command Processing
                       │
                       ▼
                Angle Controller
                       │
                       ▼
                 Rate Controller
                       │
                       ▼
                 Quad-X Motor Mixer
                       │
                       ▼
                 PWM Motor Outputs
                       │
                       ▼
            Electronic Speed Controllers
                       │
                       ▼
                 Brushless Motors
```

Meanwhile, the controller continuously estimates aircraft orientation.

```
             MPU6000 IMU
                  │
                  ▼
          Sensor Calibration
                  │
                  ▼
         Mahony Attitude Filter
                  │
                  ▼
          Roll Pitch Yaw Estimate
                  │
                  ▼
           Flight Controllers
```

---

# Layered Software Architecture

```
                    Application Layer
                            │
        ┌───────────────────┼───────────────────┐
        │                   │                   │
        ▼                   ▼                   ▼
   Flight Control      Communication        Safety
        │                   │                   │
        └───────────────────┼───────────────────┘
                            ▼
                    Estimation Layer
                            │
                            ▼
                      Device Drivers
                            │
                            ▼
              Hardware Abstraction Layer
                            │
                            ▼
                  STM32F4 Microcontroller
```

Each layer communicates only with neighbouring layers.

Higher-level software never directly accesses STM32 peripheral registers.

---

# Firmware Layers

## 1. Application Layer

The application layer coordinates the complete flight controller.

Responsibilities

- System initialization
- Flight state management
- Scheduler startup
- Task registration
- Main control loop

---

## 2. Flight Control Layer

Responsible for aircraft stabilization.

Modules include

- Angle Controller
- Rate Controller
- PID Library
- Motor Mixer

Input

- Desired aircraft attitude
- Current aircraft attitude

Output

- Motor throttle commands

---

## 3. Estimation Layer

Processes raw sensor measurements into aircraft orientation.

Modules

- IMU Calibration
- Mahony Filter
- Attitude Estimation

Input

- Accelerometer
- Gyroscope

Output

- Roll
- Pitch
- Yaw

---

## 4. Communication Layer

Handles all external communication.

Modules

- SBUS Receiver
- MSP Protocol
- Telemetry

Responsibilities

- Receive pilot commands
- Decode communication packets
- Stream telemetry
- Command processing

---

## 5. Safety Layer

Continuously monitors firmware health.

Modules

- RC Failsafe
- Arming Logic
- Watchdog

Responsibilities

- Receiver timeout detection
- Safe motor shutdown
- Fault monitoring

---

## 6. Driver Layer

Provides hardware-specific device drivers.

Drivers include

- MPU6000
- SPI
- UART
- Timer
- PWM

The driver layer hides peripheral implementation details from higher-level
software.

---

## 7. Hardware Abstraction Layer

The HAL provides standardized interfaces for STM32 peripherals.

Responsibilities

- GPIO
- SPI
- UART
- Timers
- PWM

The HAL prevents application code from directly manipulating hardware
registers.

---

# Flight Control Pipeline

```
Pilot Input

↓

SBUS Receiver

↓

Command Processing

↓

Angle Controller

↓

Desired Angular Rate

↓

Rate Controller

↓

Motor Mixer

↓

PWM Generation

↓

ESC

↓

Motors
```

---

# Sensor Processing Pipeline

```
MPU6000

↓

SPI Driver

↓

Raw Accelerometer

Raw Gyroscope

↓

Calibration

↓

Mahony Filter

↓

Attitude Estimation

↓

Roll Pitch Yaw

↓

PID Controllers
```

---

# Communication Pipeline

```
Ground Station

↓

MSP Packets

↓

UART Driver

↓

Packet Parser

↓

Command Processing

↓

Telemetry Response
```

---

# Scheduler Architecture

The firmware uses a deterministic cooperative scheduler.

```
System Tick

↓

Scheduler

↓

Task Ready?

↓

Execute Task

↓

Return

↓

Next Task
```

Typical task frequencies

| Task | Frequency |
|-------|-----------|
| IMU Update | 1000 Hz |
| Mahony Filter | 1000 Hz |
| PID Controller | 500 Hz |
| SBUS Processing | 100 Hz |
| Telemetry | 20 Hz |
| Failsafe | 100 Hz |

---

# Safety Architecture

```
Receiver Frames

↓

Timeout Monitor

↓

Signal Lost?

↓

YES

↓

Failsafe

↓

Motor Shutdown

↓

Recovery
```

Safety always has priority over normal flight operation.

---

# Module Dependencies

```
Application

↓

Flight Control

↓

Estimation

↓

Drivers

↓

HAL

↓

STM32 Hardware
```

Communication and safety operate alongside the flight-control pipeline without
directly modifying controller internals.

---

# Design Principles

The firmware follows several software engineering principles.

### Layered Architecture

Each software layer performs one engineering responsibility.

---

### Hardware Abstraction

Peripheral access is isolated inside the HAL and driver layers.

---

### Deterministic Execution

Every task executes at predefined intervals using a cooperative scheduler.

---

### Modular Design

Subsystems communicate only through public interfaces.

---

### Safety First

Fault detection and failsafe logic operate independently from flight-control
algorithms.

---

### Extensibility

The architecture allows future integration of

- GPS
- Magnetometer
- Barometer
- Optical Flow
- EKF
- MAVLink
- Autonomous Flight Modes

without redesigning the existing software.

---

# Summary

The architecture emphasizes deterministic execution, modular software design,
and clear separation of responsibilities.

Rather than implementing a collection of independent algorithms, the project
demonstrates how embedded subsystems—including hardware drivers, sensor fusion,
control algorithms, communication protocols, and safety mechanisms—interact to
form a complete flight controller suitable for real-time embedded applications.
