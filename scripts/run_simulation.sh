#!/bin/bash

# ==========================================
# FLIGHT CONTROLLER — RUN SIMULATION
# ==========================================

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  FLIGHT CONTROLLER SIMULATION           ${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Step 1: Build firmware for QEMU
echo -e "${YELLOW}[1/3]${NC} Building firmware for QEMU..."
./scripts/build_all.sh qemu

# Step 2: Start simulation
echo ""
echo -e "${YELLOW}[2/3]${NC} Starting Python simulation..."
echo -e "  ${GREEN}→${NC} Physics engine starting..."
echo -e "  ${GREEN}→${NC} Virtual sensors ready"
echo -e "  ${GREEN}→${NC} Bridge to QEMU active"

# Run simulation in background
python3 simulation/run_simulation.py &
SIM_PID=$!

# Step 3: Start telemetry viewer
echo ""
echo -e "${YELLOW}[3/3]${NC} Starting telemetry viewer..."
echo -e "  ${GREEN}→${NC} Launching real-time graphs"
echo ""

python3 telemetry_viewer/viewer.py

# Cleanup
echo ""
echo -e "${YELLOW}Stopping simulation...${NC}"
kill ${SIM_PID} 2>/dev/null

echo -e "${GREEN}Simulation complete${NC}"