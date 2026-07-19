# Flight Control Pipeline

## Overview

The flight controller continuously transforms pilot commands and sensor
measurements into motor outputs that stabilize the aircraft.

Unlike a traditional sequential program, the firmware executes this pipeline
periodically at deterministic frequencies. Each subsystem performs one
well-defined task before passing its output to the next stage.

The complete control loop executes in only a few milliseconds and repeats
hundreds of times every second.

---

# Complete Control Pipeline

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
            Desired Roll / Pitch
                    │
                    ▼
          Angle PID Controller
                    │
                    ▼
         Desired Angular Rates
                    │
                    ▼
          Rate PID Controller
                    │
                    ▼
             Motor Mixer (Quad-X)
                    │
                    ▼
              PWM Generation
                    │
                    ▼
        Electronic Speed Controllers
                    │
                    ▼
            Brushless DC Motors
```

At the same time, the aircraft state is continuously estimated.

```
             MPU6000 IMU
                  │
                  ▼
          Accelerometer Data

          Gyroscope Data
                  │
                  ▼
          Sensor Calibration
                  │
                  ▼
          Mahony Filter
                  │
                  ▼
         Roll Pitch Yaw Estimate
                  │
                  ▼
          PID Controllers
```

---

# Stage 1 — Pilot Command Acquisition

The control pipeline begins by receiving pilot commands from the radio
transmitter.

The SBUS receiver continuously transmits packets containing

- Throttle
- Roll
- Pitch
- Yaw
- Auxiliary Switches

The receiver driver

- Synchronizes SBUS frames
- Validates packets
- Decodes channel values
- Normalizes stick positions

Output

```
Throttle

Roll Command

Pitch Command

Yaw Command
```

---

# Stage 2 — Sensor Acquisition

The MPU6000 provides raw inertial measurements.

Each update produces

```
Accelerometer

Ax

Ay

Az

Gyroscope

Gx

Gy

Gz
```

The IMU driver communicates through SPI using burst transfers to reduce
communication latency.

---

# Stage 3 — Sensor Calibration

Raw measurements cannot be used directly.

Calibration removes systematic sensor errors.

Processing includes

- Gyroscope bias removal
- Accelerometer scaling
- Unit conversion

Output

```
Corrected Accelerometer

Corrected Gyroscope
```

These corrected measurements become the input to the sensor fusion algorithm.

---

# Stage 4 — Attitude Estimation

The Mahony filter combines gyroscope integration with accelerometer
correction to estimate aircraft orientation.

Input

```
Gyroscope

+

Accelerometer
```

Output

```
Roll

Pitch

Yaw
```

These values represent the estimated aircraft attitude used by the controller.

---

# Stage 5 — Angle Controller

The outer control loop compares

Desired Angle

vs

Measured Angle

Example

```
Pilot Roll Command

↓

Desired Roll Angle

↓

Compare

↓

Measured Roll

↓

Angle Error
```

A PID controller computes the desired angular velocity required to eliminate
this error.

Output

```
Desired Roll Rate

Desired Pitch Rate
```

---

# Stage 6 — Rate Controller

The inner control loop regulates angular velocity.

Input

```
Desired Roll Rate

Desired Pitch Rate

Desired Yaw Rate
```

Measured angular rates from the gyroscope are compared against these desired
values.

Another PID controller computes the required control corrections.

Output

```
Roll Correction

Pitch Correction

Yaw Correction
```

---

# Stage 7 — Motor Mixing

The controller does not command motors directly.

Instead, the mixer combines

- Throttle
- Roll Correction
- Pitch Correction
- Yaw Correction

into four individual motor commands.

Quad-X Mixing

```
Front

        M1

     \      /

      \    /

M4            M2

      /    \

     /      \

        M3

Rear
```

Example

```
Motor 1

Throttle

+ Pitch

- Roll

+ Yaw

Motor 2

Throttle

+ Pitch

+ Roll

- Yaw

Motor 3

Throttle

- Pitch

+ Roll

+ Yaw

Motor 4

Throttle

- Pitch

- Roll

- Yaw
```

The resulting outputs are constrained to safe operating limits.

---

# Stage 8 — PWM Generation

Motor commands are converted into PWM duty cycles.

Typical range

```
1000 µs

↓

Minimum Throttle

↓

1500 µs

↓

Hover

↓

2000 µs

↓

Maximum Throttle
```

PWM signals are generated using STM32 hardware timers.

---

# Stage 9 — Motor Output

The PWM outputs drive the Electronic Speed Controllers (ESCs).

Each ESC converts PWM commands into

- Motor current
- Motor torque
- Propeller thrust

The combined thrust of all four motors produces aircraft motion.

---

# Closed-Loop Feedback

The system continuously repeats this process.

```
Pilot Command

↓

Motor Output

↓

Aircraft Motion

↓

IMU Measurement

↓

Sensor Fusion

↓

Controller

↓

Motor Output
```

This continuous feedback loop allows the aircraft to remain stable even when
disturbed by external forces.

---

# Scheduler Interaction

Every stage executes according to the cooperative scheduler.

Typical execution frequencies

| Task | Frequency |
|-------|-----------|
| IMU Sampling | 1000 Hz |
| Mahony Filter | 1000 Hz |
| Angle Controller | 500 Hz |
| Rate Controller | 500 Hz |
| Motor Mixer | 500 Hz |
| SBUS Processing | 100 Hz |
| Telemetry | 20 Hz |
| Failsafe | 100 Hz |

This deterministic execution ensures consistent controller behaviour.

---

# Safety Integration

The control pipeline continuously interacts with the safety subsystem.

```
Receiver Timeout

↓

Failsafe

↓

Override Motor Commands

↓

Safe Output
```

If receiver communication is lost or invalid data is detected, the failsafe
takes priority over normal flight control.

---

# Communication Integration

The telemetry subsystem operates independently from the controller.

```
Controller State

↓

Telemetry

↓

MSP Packet

↓

Ground Station
```

This allows controller behaviour to be monitored without affecting real-time
performance.

---

# Timing Requirements

A typical control iteration follows this sequence.

```
Read IMU

↓

Estimate Attitude

↓

Read Receiver

↓

Angle PID

↓

Rate PID

↓

Motor Mixing

↓

Generate PWM

↓

Transmit Telemetry
```

Each iteration completes before the next scheduled update begins.

Maintaining deterministic execution is essential because PID controllers
assume a fixed sampling interval.

---

# Design Philosophy

The flight control pipeline follows several engineering principles.

### Deterministic Execution

Every stage executes at predefined intervals.

---

### Layered Processing

Each subsystem performs one engineering responsibility.

---

### Closed-Loop Feedback

Every motor command is computed using continuously updated sensor feedback.

---

### Hardware Independence

Flight-control algorithms remain independent of STM32 peripheral
implementation.

---

### Safety First

Failsafe logic always has priority over normal controller operation.

---

# Summary

The flight control pipeline converts pilot commands into stable aircraft
motion through a sequence of independent processing stages.

Beginning with receiver input and sensor acquisition, the firmware performs
sensor fusion, cascaded PID control, motor mixing, and PWM generation before
driving the Electronic Speed Controllers.

By separating estimation, control, communication, and safety into modular
components executed by a deterministic scheduler, the firmware achieves a
maintainable and scalable architecture suitable for real-time embedded flight
control applications.
