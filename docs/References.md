---

# References

The design and implementation of this project are based on established
principles from embedded systems, control engineering, sensor fusion, and
flight control software. The following resources were used for understanding
theoretical concepts, communication protocols, and embedded firmware design.

---

## Embedded Systems

- Joseph Yiu, **The Definitive Guide to ARM® Cortex®-M3 and Cortex®-M4 Processors**, 3rd Edition.
- Jonathan Valvano, **Embedded Systems: Real-Time Interfacing to ARM Cortex-M Microcontrollers**.
- Michael Barr & Anthony Massa, **Programming Embedded Systems**.
- Jack Ganssle, **The Art of Designing Embedded Systems**.

---

## Control Systems

- Karl J. Åström and Richard M. Murray, **Feedback Systems: An Introduction for Scientists and Engineers**.
- Gene F. Franklin, J. David Powell, Abbas Emami-Naeini, **Feedback Control of Dynamic Systems**.
- Katsuhiko Ogata, **Modern Control Engineering**.

---

## Sensor Fusion & IMU Processing

- Robert Mahony, Tarek Hamel, and Jean-Michel Pflimlin,
  **Nonlinear Complementary Filters on the Special Orthogonal Group**.
- Sebastian Madgwick,
  **An Efficient Orientation Filter for Inertial and Inertial/Magnetic Sensor Arrays**.
- InvenSense,
  **MPU-6000/MPU-6050 Product Specification**.
- InvenSense,
  **MPU-6000 Register Map and Register Descriptions**.

---

## UAV Flight Control

- Randal W. Beard and Timothy W. McLain,
  **Small Unmanned Aircraft: Theory and Practice**.
- Austin, R.,
  **Unmanned Aircraft Systems**.
- Stevens, Lewis & Johnson,
  **Aircraft Control and Simulation**.

---

## Communication Protocols

### SBUS

- Futaba Corporation,
  **SBUS Serial Bus Communication Protocol**.

### MultiWii Serial Protocol (MSP)

- MultiWii Project Documentation
- Betaflight MSP Documentation

---

## Open-Source Flight Controller Projects

The project architecture was inspired by concepts used in professional
open-source flight control software.

- PX4 Autopilot
- ArduPilot
- Betaflight
- Cleanflight
- INAV

The firmware presented in this repository is an independent implementation
developed from first principles and is **not derived from or copied from
these projects**.

---

## STM32 Documentation

- STM32F405/407 Reference Manual (RM0090)
- STM32F4 Programming Manual
- STM32Cube HAL Documentation
- STM32 Application Notes

---

## Development Tools

- GNU Arm Embedded Toolchain
- GNU Make
- GDB
- STM32CubeProgrammer
- Python 3
- NumPy
- Matplotlib

---

## Mathematical References

The implementation follows standard engineering concepts from

- Rigid Body Dynamics
- Linear Control Systems
- PID Control Theory
- Rotation Matrices
- Quaternions
- Digital Signal Processing
- Numerical Integration

---

## Standards & Best Practices

- MISRA C Guidelines (reference only)
- ARM Embedded Application Notes
- IEEE Embedded Systems Publications
- Embedded Artistry Articles

---

## Acknowledgements

This project was developed as a personal learning and engineering exercise to
gain practical experience in

- Bare-Metal Embedded Systems
- Flight Control Software
- Real-Time Programming
- Sensor Fusion
- Control Systems
- Embedded Communication Protocols
- Safety-Critical Firmware Design

The implementation is intended for educational and portfolio purposes and is
designed to demonstrate software architecture, engineering decision-making,
and embedded firmware development practices rather than serve as a certified
flight controller for operational aircraft.
