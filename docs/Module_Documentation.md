---

# Module Documentation

The firmware is organized into independent software modules, each responsible
for a single engineering function. This modular architecture minimizes coupling,
simplifies debugging, and allows new functionality to be integrated without
modifying the entire codebase.

---

## Hardware Abstraction Layer (`hal/`)

**Purpose**

Provides a hardware-independent interface to STM32 peripherals.

The HAL isolates application logic from register-level programming, allowing
higher-level modules to communicate with hardware through well-defined APIs.

### Responsibilities

- GPIO configuration
- SPI communication
- UART communication
- Timer configuration
- PWM generation
- Clock initialization

### Key Interfaces

```c
hal_spi_transfer()

hal_uart_send()

hal_pwm_write()

hal_timer_get_ticks()
```

### Used By

- IMU Driver
- SBUS Receiver
- MSP Protocol
- Motor Output
- Scheduler

---

## Device Drivers (`drivers/`)

**Purpose**

Implements low-level communication with external hardware devices.

Each driver is responsible only for communicating with its associated hardware
and exposing a clean software interface to the remainder of the firmware.

### Modules

- MPU6000 IMU
- SBUS Receiver
- MSP Communication

### Responsibilities

- Device initialization
- Register access
- Data acquisition
- Packet decoding
- Error detection

### Example Flow

```
Application

↓

MPU6000 Driver

↓

SPI Driver

↓

STM32 SPI Peripheral
```

---

## Scheduler (`scheduler/`)

**Purpose**

Executes every firmware task at deterministic frequencies.

Rather than continuously executing functions inside an infinite loop, the
scheduler ensures each subsystem operates according to predefined update
intervals.

### Responsibilities

- Task registration
- Periodic execution
- Timing management
- Frequency control

### Typical Tasks

| Task | Frequency |
|--------|----------|
| IMU Update | 1000 Hz |
| Sensor Fusion | 1000 Hz |
| PID Controller | 500 Hz |
| SBUS Processing | 100 Hz |
| Telemetry | 20 Hz |
| Failsafe | 100 Hz |

### Benefits

- Predictable timing
- Easy debugging
- Low overhead
- Deterministic execution

---

## Estimation (`estimation/`)

**Purpose**

Converts raw IMU measurements into stable aircraft orientation.

The estimation layer combines gyroscope and accelerometer measurements using
the Mahony filter to estimate roll, pitch, and yaw.

### Components

- IMU Calibration
- Mahony Filter
- Attitude Estimation

### Processing Pipeline

```
Raw IMU Data

↓

Calibration

↓

Mahony Filter

↓

Roll

Pitch

Yaw
```

### Responsibilities

- Gyroscope bias removal
- Accelerometer scaling
- Sensor fusion
- Attitude estimation

---

## Control (`control/`)

**Purpose**

Generates motor commands required to stabilize the aircraft.

The control subsystem implements a cascaded PID architecture similar to those
used in modern UAV flight controllers.

### Components

- Angle Controller
- Rate Controller
- PID Library
- Motor Mixer

### Control Pipeline

```
Pilot Commands

↓

Angle Controller

↓

Desired Angular Rate

↓

Rate Controller

↓

Motor Mixer

↓

Motor Outputs
```

### Responsibilities

- Angle stabilization
- Rate stabilization
- Integral windup protection
- Motor mixing

---

## Communication (`communication/`)

**Purpose**

Handles communication between the flight controller and external devices.

This subsystem provides receiver input, telemetry transmission, and debugging
interfaces while remaining independent of flight-control logic.

### Components

- SBUS Decoder
- MSP Protocol
- Telemetry

### Responsibilities

- Receive pilot commands
- Parse communication packets
- Transmit flight information
- Validate packet integrity

### Supported Protocols

- SBUS
- MSP

---

## Safety (`safety/`)

**Purpose**

Monitors firmware health and protects the aircraft during abnormal operating
conditions.

The safety subsystem operates independently from the flight controller to
ensure predictable behaviour during failures.

### Components

- RC Failsafe
- Arming Logic
- Watchdog

### Responsibilities

- Receiver timeout detection
- Signal validation
- Safe state transitions
- Motor shutdown protection

### Example State Machine

```
Disarmed

↓

Armed

↓

Flying

↓

Signal Loss

↓

Failsafe

↓

Recovery / Disarm
```

---

## Telemetry (`telemetry/`)

**Purpose**

Streams internal flight controller information for debugging and analysis.

Telemetry allows engineers to visualize controller behaviour without modifying
embedded firmware.

### Transmitted Data

- Roll
- Pitch
- Yaw
- Gyroscope
- Accelerometer
- Receiver Channels
- PID Outputs
- Motor Commands
- System Status

### Benefits

- Real-time visualization
- Easier controller tuning
- Faster debugging
- Performance analysis

---

## Simulation (`simulation/`)

**Purpose**

Provides a software-only environment for validating control algorithms before
hardware deployment.

The simulator models basic multirotor dynamics and communicates with the
embedded firmware through a telemetry bridge.

### Components

- Flight Physics
- Sensor Models
- Telemetry Bridge
- Visualization

### Responsibilities

- Aircraft dynamics simulation
- IMU emulation
- Flight visualization
- Algorithm validation

---

## Tests (`tests/`)

**Purpose**

Validates firmware behaviour through automated unit and integration tests.

Testing individual modules independently improves software reliability and
reduces debugging time.

### Test Coverage

- Scheduler
- PID Controller
- Mahony Filter
- Motor Mixer
- SBUS Decoder
- MSP Protocol
- RC Failsafe

### Example

```bash
python tests/run_all.py
```

Expected Output

```
Scheduler ........ PASS

PID Controller ... PASS

Motor Mixer ...... PASS

Failsafe ......... PASS
```

---

## Documentation (`docs/`)

**Purpose**

Contains technical documentation describing firmware architecture,
implementation details, mathematical models, and engineering decisions.

### Documents

- Architecture Overview
- Control Pipeline
- Sensor Fusion
- Communication Protocols
- Scheduler Design
- Engineering Retrospective

---

## Module Interaction

```
                    Application
                         │
                         ▼
                  Flight Controllers
                         │
        ┌────────────────┼────────────────┐
        ▼                ▼                ▼
   Estimation     Communication      Safety
        │                │                │
        └────────────────┼────────────────┘
                         ▼
                      Drivers
                         ▼
                        HAL
                         ▼
                  STM32 Hardware
```

---

## Design Philosophy

Every module in the project follows three fundamental engineering principles.

### Single Responsibility

Each module performs one clearly defined engineering task.

### Loose Coupling

Modules communicate only through public interfaces rather than directly
accessing each other's internal implementation.

### High Cohesion

Related functionality remains grouped together, making the firmware easier
to understand, maintain, and extend.

This modular organization allows future features such as GPS navigation,
barometer integration, MAVLink communication, and autonomous flight modes
to be added with minimal changes to the existing codebase.
