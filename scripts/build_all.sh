#!/bin/bash

# ==========================================
# FLIGHT CONTROLLER — BUILD ALL SCRIPT
# ==========================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  FLIGHT CONTROLLER — BUILD SYSTEM      ${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

if [ "$1" == "qemu" ] || [ "$1" == "" ]; then
    TARGET="qemu"
    TARGET_FLAG="-DTARGET_QEMU=1"
    echo -e "${GREEN}[TARGET]${NC} QEMU Simulation"
elif [ "$1" == "stm32" ]; then
    TARGET="stm32"
    TARGET_FLAG="-DTARGET_STM32=1"
    echo -e "${GREEN}[TARGET]${NC} STM32F405 Hardware"
else
    echo -e "${RED}[ERROR]${NC} Unknown target: $1"
    echo "Usage: $0 [qemu|stm32]"
    exit 1
fi

BUILD_DIR="build/${TARGET}"
mkdir -p ${BUILD_DIR}

echo ""
echo -e "${YELLOW}[BUILD]${NC} Compiling flight controller firmware..."

FIRMWARE_SOURCES=(
    "firmware/src/main.c"
    "firmware/src/system/clock_config.c"
    "firmware/src/system/scheduler.c"
    "firmware/src/hal/hal_spi.c"
    "firmware/src/hal/hal_uart.c"
    "firmware/src/hal/hal_tim.c"
    "firmware/src/hal/hal_adc.c"
    "firmware/src/hal/simulated_hardware.c"
    "firmware/src/estimation/mahony_filter.c"
    "firmware/src/control/pid.c"
    "firmware/src/control/rate_controller.c"
    "firmware/src/control/angle_controller.c"
    "firmware/src/control/mixer.c"
    "firmware/src/communication/sbus_decoder.c"
    "firmware/src/communication/msp_protocol.c"
    "firmware/src/failsafe/rc_failsafe.c"
    "firmware/src/utils/math_utils.c"
    "firmware/src/utils/circular_buffer.c"
)

INCLUDE_DIRS=(
    "-Ifirmware"
    "-Ifirmware/config"
    "-Ifirmware/src"
    "-Ifirmware/src/system"
    "-Ifirmware/src/hal"
    "-Ifirmware/src/drivers/imu"
    "-Ifirmware/src/estimation"
    "-Ifirmware/src/control"
    "-Ifirmware/src/communication"
    "-Ifirmware/src/failsafe"
    "-Ifirmware/src/utils"
)

gcc -o ${BUILD_DIR}/flight_controller \
    "${FIRMWARE_SOURCES[@]}" \
    "${INCLUDE_DIRS[@]}" \
    ${TARGET_FLAG} \
    -Wall -Wextra -O2 \
    -lm

if [ $? -eq 0 ]; then
    echo -e "${GREEN}[SUCCESS]${NC} Firmware built: ${BUILD_DIR}/flight_controller"
else
    echo -e "${RED}[FAILED]${NC} Firmware compilation failed"
    exit 1
fi

echo ""
echo -e "${YELLOW}[BUILD]${NC} Building unit tests..."

TESTS=(
    "test_pid:firmware/src/control/pid.c tests/test_pid.c -Ifirmware/src/control -lm"
    "test_mixer:firmware/src/control/mixer.c tests/test_mixer.c -Ifirmware/src/control -lm"
    "test_sbus:firmware/src/communication/sbus_decoder.c tests/test_sbus.c -Ifirmware/src/communication -lm"
    "test_mahony:firmware/src/estimation/mahony_filter.c tests/test_mahony.c -Ifirmware/src/estimation -lm"
    "test_failsafe:firmware/src/failsafe/rc_failsafe.c tests/test_rc_failsafe.c -Ifirmware/src/failsafe -lm"
    "test_scheduler:firmware/src/system/scheduler.c tests/test_scheduler.c -Ifirmware/src/system -lm"
    "test_rate:firmware/src/control/pid.c firmware/src/control/rate_controller.c tests/test_rate_controller.c -Ifirmware/src/control -lm"
    "test_angle:firmware/src/control/pid.c firmware/src/control/angle_controller.c tests/test_angle_controller.c -Ifirmware/src/control -lm"
    "test_msp:firmware/src/communication/msp_protocol.c tests/test_msp.c -Ifirmware/src/communication -lm"
)

TEST_PASSED=0
TEST_FAILED=0

for test_info in "${TESTS[@]}"; do
    TEST_NAME="${test_info%%:*}"
    TEST_SOURCES="${test_info#*:}"
    echo -e "  Building ${TEST_NAME}..."
    gcc -o ${BUILD_DIR}/${TEST_NAME} ${TEST_SOURCES} -Wall -Wextra -O2
    if [ $? -eq 0 ]; then
        echo -e "    ${GREEN}✓ Built${NC}"
    else
        echo -e "    ${RED}✗ Failed${NC}"
    fi
done

echo ""
echo -e "${YELLOW}[TEST]${NC} Running unit tests..."
echo ""

for test_info in "${TESTS[@]}"; do
    TEST_NAME="${test_info%%:*}"
    if [ -f "${BUILD_DIR}/${TEST_NAME}" ]; then
        echo -e "${BLUE}─── ${TEST_NAME} ───${NC}"
        if ${BUILD_DIR}/${TEST_NAME}; then
            TEST_PASSED=$((TEST_PASSED + 1))
            echo -e "${GREEN}  ✓ ${TEST_NAME} PASSED${NC}"
        else
            TEST_FAILED=$((TEST_FAILED + 1))
            echo -e "${RED}  ✗ ${TEST_NAME} FAILED${NC}"
        fi
        echo ""
    fi
done

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  BUILD SUMMARY                          ${NC}"
echo -e "${BLUE}========================================${NC}"
echo -e "  Target:     ${GREEN}${TARGET}${NC}"
echo -e "  Firmware:   ${GREEN}${BUILD_DIR}/flight_controller${NC}"
echo -e "  Tests:      ${GREEN}${TEST_PASSED} passed${NC}, ${RED}${TEST_FAILED} failed${NC}"
echo ""

if [ ${TEST_FAILED} -eq 0 ]; then
    echo -e "${GREEN}  ALL TESTS PASSED ✓${NC}"
else
    echo -e "${RED}  SOME TESTS FAILED ✗${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}  Next steps:${NC}"
echo -e "    Run simulation:  ${YELLOW}./scripts/run_simulation.sh${NC}"
echo -e "    View telemetry:  ${YELLOW}python3 telemetry_viewer/viewer.py${NC}"
echo ""