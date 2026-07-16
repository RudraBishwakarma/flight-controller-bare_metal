# Flight Controller Simulation

## Overview
Python-based simulation that connects to QEMU-emulated firmware.
Closes the control loop: firmware → motors → physics → sensors → firmware.

## Components
- **drone_physics/**: 6DOF rigid body dynamics + motor models
- **sensors/**: Virtual IMU with realistic noise, bias, and drift
- **bridge/**: Unix socket bridge to QEMU virtual serial port

## Usage
1. Start QEMU with firmware: `./scripts/run_qemu.sh`
2. Start simulation: `python3 run_simulation.py`
3. Connect Mission Planner to UDP:14550

## Standalone Mode
If QEMU is not running, the simulation runs in standalone mode
with fixed motor commands for physics testing.