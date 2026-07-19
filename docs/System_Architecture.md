---

## System Architecture

The firmware follows a layered architecture commonly used in modern embedded
control systems. Each layer has a single engineering responsibility, allowing
individual modules to be developed, tested, and maintained independently.

```
                         User Commands
                               │
                               ▼
                       SBUS Receiver Driver
                               │
                               ▼
                     Flight Mode / Input Processing
                               │
                               ▼
                     Attitude Controller (PID)
                               │
                               ▼
                       Rate Controller (PID)
                               │
                               ▼
                         Quad-X Motor Mixer
                               │
                               ▼
                          PWM Generation
                               │
                               ▼
                      Electronic Speed Controllers
                               │
                               ▼
                         Brushless Motors


                  ┌─────────────────────────────┐
                  │      Sensor Subsystem        │
                  │                             │
                  │     MPU6000 IMU (SPI)       │
                  │            │                │
                  │            ▼                │
                  │   Sensor Calibration        │
                  │            │                │
                  │            ▼                │
                  │   Mahony Attitude Filter    │
                  └─────────────────────────────┘

                              ▲
                              │
                  Deterministic Scheduler
                              │
                              ▼

      Telemetry ─── MSP Protocol ─── Failsafe Monitor
                              │
                              ▼
                       Hardware Abstraction Layer
                              │
                              ▼
                       STM32F4 Microcontroller
```

### Software Layers

| Layer | Responsibility |
|--------|----------------|
| **Application** | Flight modes, control pipeline, and system coordination. |
| **Control** | Cascaded PID controllers and motor mixing. |
| **Estimation** | Sensor fusion and attitude estimation using the Mahony filter. |
| **Communication** | SBUS receiver decoding, MSP protocol, and telemetry. |
| **Safety** | RC failsafe monitoring and system fault handling. |
| **Scheduler** | Deterministic cooperative task execution. |
| **Drivers** | IMU, UART, SPI, PWM, timers, and GPIO interfaces. |
| **HAL** | Hardware abstraction over STM32 peripherals. |

The architecture separates hardware-specific functionality from flight logic,
making the firmware easier to debug, extend, and port to future hardware.

---

## Project Structure

```
flight-controller/
│
├── application/              # Main firmware entry point
│   ├── main.c
│   └── flight_controller.c
│
├── hal/                      # Hardware abstraction layer
│   ├── gpio/
│   ├── spi/
│   ├── uart/
│   ├── timer/
│   └── pwm/
│
├── drivers/                  # Device drivers
│   ├── imu/
│   │   ├── mpu6000.c
│   │   └── mpu6000.h
│   ├── sbus/
│   └── msp/
│
├── scheduler/                # Cooperative scheduler
│   ├── scheduler.c
│   └── scheduler.h
│
├── estimation/               # State estimation
│   ├── mahony_filter.c
│   ├── calibration.c
│   └── attitude.c
│
├── control/                  # Flight control algorithms
│   ├── pid.c
│   ├── angle_controller.c
│   ├── rate_controller.c
│   └── mixer.c
│
├── communication/            # External communication
│   ├── telemetry.c
│   ├── msp_protocol.c
│   └── sbus_decoder.c
│
├── safety/                   # Safety mechanisms
│   ├── failsafe.c
│   ├── arming.c
│   └── watchdog.c
│
├── simulation/               # Python flight simulator
│   ├── physics.py
│   ├── telemetry_bridge.py
│   └── visualizer.py
│
├── tests/                    # Unit and integration tests
│   ├── test_pid.py
│   ├── test_scheduler.py
│   ├── test_failsafe.py
│   └── run_all.py
│
├── docs/                     # Project documentation
│   ├── architecture.md
│   ├── control_pipeline.md
│   ├── software_design.md
│   └── scheduler.md
│
├── tools/                    # Utility scripts
│
├── Makefile
├── README.md
└── ENGINEERING_RETROSPECTIVE.md
```

### Directory Overview

| Directory | Purpose |
|-----------|---------|
| **application/** | Firmware initialization and high-level control flow. |
| **hal/** | Hardware abstraction for STM32 peripherals. |
| **drivers/** | Low-level drivers for sensors and communication peripherals. |
| **scheduler/** | Deterministic cooperative scheduler implementation. |
| **estimation/** | IMU calibration and Mahony attitude estimation. |
| **control/** | PID controllers, motor mixer, and flight control logic. |
| **communication/** | SBUS, MSP, and telemetry interfaces. |
| **safety/** | Failsafe logic, watchdog, and arming state machine. |
| **simulation/** | Python-based flight dynamics simulator and visualization tools. |
| **tests/** | Automated unit and integration tests. |
| **docs/** | Technical documentation and design notes. |
| **tools/** | Build and development helper scripts. |

The project structure follows a **modular embedded software architecture**, where every directory owns a single engineering responsibility. This minimizes coupling between subsystems and allows new features—such as GPS navigation, EKF state estimation, or MAVLink communication—to be integrated with minimal impact on the existing codebase.
