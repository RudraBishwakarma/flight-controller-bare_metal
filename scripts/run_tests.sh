#!/bin/bash

# ==========================================
# FLIGHT CONTROLLER — RUN ALL TESTS
# ==========================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  RUNNING ALL UNIT TESTS                 ${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

BUILD_DIR="build/qemu"

# Check if build exists
if [ ! -d "${BUILD_DIR}" ]; then
    echo -e "${YELLOW}Build directory not found. Building...${NC}"
    ./scripts/build_all.sh qemu
fi

# List of all tests
TESTS=(
    "test_pid"
    "test_mixer"
    "test_sbus"
    "test_mahony"
    "test_failsafe"
    "test_scheduler"
    "test_rate"
    "test_angle"
    "test_msp"
)

PASSED=0
FAILED=0

for test in "${TESTS[@]}"; do
    echo -e "${BLUE}─── ${test} ───${NC}"
    
    if [ -f "${BUILD_DIR}/${test}" ]; then
        if ${BUILD_DIR}/${test}; then
            PASSED=$((PASSED + 1))
            echo -e "${GREEN}✓ ${test} PASSED${NC}"
        else
            FAILED=$((FAILED + 1))
            echo -e "${RED}✗ ${test} FAILED${NC}"
        fi
    else
        echo -e "${RED}✗ ${test} NOT FOUND${NC}"
        FAILED=$((FAILED + 1))
    fi
    echo ""
done

echo -e "${BLUE}========================================${NC}"
echo -e "  Results: ${GREEN}${PASSED} passed${NC}, ${RED}${FAILED} failed${NC}"
echo -e "${BLUE}========================================${NC}"

if [ ${FAILED} -gt 0 ]; then
    exit 1
fi