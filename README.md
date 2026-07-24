# Bare-Metal Flight Controller

A modular bare-metal flight controller firmware for STM32F4-class microcontrollers designed to
demonstrate real-time embedded systems, flight stabilization, sensor fusion, communication
protocols, and safety engineering. The project implements the complete flight control pipeline
from sensor acquisition to motor output while emphasizing deterministic execution, modular
software architecture, and engineering best practices.

---

## What this project proves

- You can design deterministic bare-metal firmware without relying on an RTOS.
- You can implement a complete flight control pipeline from IMU acquisition to motor control.
- You can integrate sensor fusion algorithms for real-time attitude estimation.
- You can develop modular embedded software using layered architecture and hardware abstraction.
- You can implement industry-standard communication protocols used in UAV systems.
- You can engineer safety-critical firmware using failsafe mechanisms and state machines.
- You can validate embedded control algorithms through simulation and structured testing.

---

## Architecture at a glance

- **Hardware Abstraction Layer (HAL):** SPI, UART, Timers, PWM and GPIO interfaces.
- **Scheduler:** Deterministic cooperative task scheduler.
- **Sensor Driver:** MPU6000 IMU acquisition and calibration.
- **Sensor Fusion:** Mahony attitude estimation algorithm.
- **Flight Control:** Cascaded PID angle and rate controllers.
- **Motor Mixer:** Quad-X motor output computation.
- **Receiver:** SBUS protocol decoding.
- **Communication:** MultiWii Serial Protocol (MSP).
- **Safety:** RC failsafe state machine and fault monitoring.
- **Telemetry:** Real-time flight data streaming.
- **Simulation:** Python-based flight dynamics simulation and visualization.

Primary references:

- `docs/architecture.md`
- `docs/control_pipeline.md`
- `docs/software_design.md`
- `ENGINEERING_RETROSPECTIVE.md`

---

## 🚀 Quick Start

### Prerequisites

- **GCC** (for PC simulation/testing)
- **Python 3.x** with `numpy` and `matplotlib` (for telemetry viewer)
- **ARM GCC toolchain** `arm-none-eabi-gcc` (only for STM32 hardware)

### Build Firmware

```bash
# Clone the repository
git clone https://github.com/YOUR_USERNAME/flight-controller-bare-metal.git
cd flight-controller-bare-metal

# Build firmware and run all tests (UCRT64 / Git Bash / Linux)
./scripts/build_all.sh

```
### Run Unit Tests
```bash
# Run all 9 test suites (63 individual tests)
./scripts/run_tests.sh

```

### Run Flight Controller (Simulation)
```bash
# Run the firmware directly (QEMU target)
./build/qemu/flight_controller

```

### Run Unit Tests
```bash
# Run all 9 test suites (63 individual tests)
./scripts/run_tests.sh

```

### Run Telemetry Viewer
```bash
# Windows CMD:
python telemetry_viewer/viewer.py

# Linux/Mac:
python3 telemetry_viewer/viewer.py

```

### Run Python Simulation
```bash
# Windows CMD:
python simulation/run_simulation.py

# Linux/Mac:
python3 simulation/run_simulation.py

```
Expected output 
Initializing Flight Controller...

MPU6000 Initialized

Mahony Filter Started

PID Controllers Initialized

Scheduler Running

System Ready
```

Telemetry data and simulation plots should appear automatically during execution.

---

## Supported Features

- Bare-Metal STM32 Firmware
- Hardware Abstraction Layer
- Deterministic Cooperative Scheduler
- MPU6000 SPI Driver
- Sensor Calibration
- Mahony Attitude Filter
- Roll, Pitch and Yaw Estimation
- Cascaded PID Controllers
- Integral Windup Protection
- Quad-X Motor Mixer
- PWM Motor Outputs
- SBUS Receiver Decoder
- MSP Communication Protocol
- RC Failsafe State Machine
- Telemetry Interface
- Modular Software Architecture
- Python Flight Simulator
- Unit Testing Framework

---

## Flight Control Pipeline

```
Pilot Commands

↓

SBUS Receiver

↓

Attitude Controller

↓

Rate Controller

↓

Motor Mixer

↓

PWM Outputs

↓

Electronic Speed Controllers

↓

Brushless Motors
```

---

## Software Architecture

```
Application

↓

Flight Controllers

↓

Sensor Fusion

↓

Drivers

↓

Hardware Abstraction Layer

↓

STM32 Hardware
```

---

## Module Map

- `application/` – Flight controller entry point.
- `hal/` – Hardware abstraction layer.
- `drivers/` – MPU6000, UART, SPI, Timer and PWM drivers.
- `scheduler/` – Cooperative real-time scheduler.
- `estimation/` – Mahony attitude estimation.
- `control/` – PID controllers and motor mixer.
- `communication/` – SBUS and MSP protocol implementation.
- `telemetry/` – Flight data transmission.
- `safety/` – RC failsafe and fault handling.
- `simulation/` – Python flight simulator.
- `tests/` – Unit and integration tests.
- `docs/` – Design documentation and architecture.

---

## Engineering Decisions and Trade-offs

- **Bare-Metal Architecture:** Chosen over an RTOS to provide deterministic execution with minimal overhead.
- **Cooperative Scheduler:** Simpler timing analysis while maintaining predictable task execution.
- **Hardware Abstraction Layer:** Separates application logic from STM32 peripherals.
- **SPI-Based IMU Communication:** Lower latency and higher throughput than I²C.
- **Mahony Filter:** Provides an effective balance between computational efficiency and estimation accuracy.
- **Cascaded PID Controllers:** Separates attitude and rate control for improved stability.
- **Quad-X Mixing Matrix:** Decouples control algorithms from airframe geometry.
- **SBUS Protocol:** Reduces hardware complexity while supporting multiple control channels.
- **MSP Communication:** Enables structured telemetry and future configuration support.
- **State Machine Failsafe:** Ensures predictable behaviour during receiver communication loss.

---

## Engineering Validation

The firmware validates embedded control algorithms by verifying that

- Sensor measurements remain stable after calibration.
- Attitude estimates remain consistent during continuous operation.
- PID controllers maintain stable closed-loop behaviour.
- Scheduler executes tasks at deterministic frequencies.
- Motor outputs remain within safe operating limits.
- SBUS frames are decoded correctly.
- MSP packets pass checksum validation.
- Failsafe activates correctly after receiver timeout.
- Simulation behaviour matches expected flight dynamics.

---

## Future Roadmap

- Extended Kalman Filter (EKF)
- Magnetometer Integration
- Barometer Support
- GPS Navigation
- Altitude Hold
- Position Hold
- Optical Flow
- Autonomous Flight Modes
- Mission Planner Support
- MAVLink Communication
- Blackbox Flight Logging
- Adaptive PID Tuning
- Quad+, Hexa and Octocopter Mixing
- Hardware-in-the-Loop (HIL) Simulation
- FreeRTOS Comparison Build

---

## Evidence and Results

The project produces

- Real-Time Attitude Estimates
- PID Controller Outputs
- Motor PWM Commands
- Receiver Channel Data
- Telemetry Streams
- Simulation Graphs
- Unit Test Results

Future versions will include

- Flight Log Analysis
- Controller Frequency Measurements
- CPU Utilization Profiling
- Latency Benchmarks
- Hardware Flight Validation
- Automated Regression Testing

---

## Limits and Boundaries

- Current implementation focuses on attitude stabilization.
- GPS navigation and autonomous flight remain future enhancements.
- The firmware currently targets STM32F4-class microcontrollers.
- Only Quad-X motor mixing is implemented.
- Advanced state estimation (EKF) is planned but not yet integrated.
- Hardware-in-the-loop testing remains future work.

---

## Deep-Dive Documentation

- `docs/architecture.md` – Software architecture.
- `docs/control_pipeline.md` – Flight control algorithms.
- `docs/scheduler.md` – Real-time scheduler design.
- `docs/sensor_fusion.md` – Mahony filter implementation.
- `docs/protocols.md` – SBUS and MSP communication.
- `ENGINEERING_RETROSPECTIVE.md` – Engineering decisions, trade-offs and lessons learned.

---

## Technologies Used

- C
- Python
- GNU Arm Embedded Toolchain
- STM32
- Make
- NumPy
- Matplotlib

---

## Author

**Rudra Bishwakarma**

B.Tech Electronics & Communication Engineering

Jaypee University of Engineering & Technology

GitHub: https://github.com/RudraBishwakarma
