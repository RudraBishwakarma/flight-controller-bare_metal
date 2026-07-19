---

# Build & Usage

The project is built using the GNU Arm Embedded Toolchain and a Makefile-based
build system. Firmware can be compiled for STM32 hardware, while the Python
simulation environment allows control algorithms to be validated without a
physical flight controller.

---

## Prerequisites

### Embedded Toolchain

Install the following software before building the firmware.

- GNU Arm Embedded Toolchain (`arm-none-eabi-gcc`)
- GNU Make
- Python 3.x
- STM32CubeProgrammer (for flashing)
- ST-Link (recommended)

Verify the installation

```bash
arm-none-eabi-gcc --version
make --version
python --version
```

---

## Clone the Repository

```bash
git clone https://github.com/RudraBishwakarma/flight-controller.git

cd flight-controller
```

---

## Build the Firmware

Compile the complete firmware

```bash
make
```

Successful compilation generates

```
build/

├── flight_controller.elf

├── flight_controller.bin

└── flight_controller.hex
```

---

## Clean Build Files

Remove all generated objects

```bash
make clean
```

---

## Rebuild

```bash
make clean

make
```

---

## Flash to STM32

Using STM32CubeProgrammer

```bash
STM32_Programmer_CLI \
-c port=SWD \
-w build/flight_controller.hex \
-v \
-rst
```

Or flash manually using the STM32CubeProgrammer GUI.

---

## Run Unit Tests

The repository includes Python-based tests for validating individual
firmware components.

Execute all tests

```bash
python tests/run_all.py
```

Run individual tests

```bash
python tests/test_pid.py

python tests/test_scheduler.py

python tests/test_failsafe.py
```

Expected output

```
====================================

Running Unit Tests

====================================

PID Controller ........ PASS

Scheduler ............. PASS

Failsafe .............. PASS

Motor Mixer ........... PASS

====================================

All Tests Passed
```

---

## Run the Flight Simulator

The simulator models basic flight dynamics and allows controller behaviour
to be evaluated before deploying to hardware.

```bash
python simulation/main.py
```

Expected output

```
Initializing Simulation...

Loading Flight Controller...

Initializing IMU...

Starting Scheduler...

Simulation Running
```

Telemetry graphs and attitude visualization should appear automatically.

---

## Runtime Workflow

```
Power On

↓

Initialize HAL

↓

Initialize Drivers

↓

Calibrate IMU

↓

Initialize Mahony Filter

↓

Initialize PID Controllers

↓

Start Scheduler

↓

Receive SBUS Commands

↓

Estimate Attitude

↓

Compute PID Output

↓

Mix Motor Commands

↓

Generate PWM Signals

↓

Transmit Telemetry
```

---

## Typical Development Workflow

```
Modify Source Code

↓

Compile Firmware

↓

Run Unit Tests

↓

Execute Simulation

↓

Verify Telemetry

↓

Flash STM32 Hardware

↓

Hardware Testing
```

This workflow allows algorithms to be validated in simulation before
deployment, reducing debugging time and improving firmware reliability.

---

## Build Targets

| Command | Description |
|----------|-------------|
| `make` | Build the complete firmware |
| `make clean` | Remove generated build files |
| `make flash` | Flash firmware to STM32 (if supported) |
| `make size` | Display firmware memory usage |
| `make help` | Show available build targets |

---

## Expected Build Output

```
========================================

Building Flight Controller Firmware

========================================

Compiling Drivers...

Compiling Scheduler...

Compiling Sensor Fusion...

Compiling Controllers...

Compiling Communication...

Compiling Safety Modules...

Linking Firmware...

Generating Binary...

========================================

Build Successful

Flash : 78 KB

RAM   : 18 KB

========================================
```

---

## Supported Development Environment

| Component | Version |
|----------|---------|
| Language | C17 |
| Compiler | GNU Arm Embedded GCC |
| Build System | GNU Make |
| Target MCU | STM32F405 / STM32F4 Series |
| Debugger | ST-Link / GDB |
| Simulator | Python 3.x |
| Visualization | Matplotlib |

---

## Continuous Validation

The recommended development cycle is

```
Write Code

↓

Compile

↓

Run Unit Tests

↓

Run Simulation

↓

Analyze Telemetry

↓

Flash Hardware

↓

Verify Flight Behaviour
```

Following this iterative workflow allows software defects to be identified
early while maintaining deterministic firmware behaviour throughout
development.
