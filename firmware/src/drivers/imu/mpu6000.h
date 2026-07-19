#ifndef MPU6000_H
#define MPU6000_H

#include <stdint.h>
#include <stdbool.h>

// ==========================================
// MPU6000 REGISTER ADDRESSES
// ==========================================

#define MPU6000_RA_SELF_TEST_X_GYRO    0x00
#define MPU6000_RA_SELF_TEST_Y_GYRO    0x01
#define MPU6000_RA_SELF_TEST_Z_GYRO    0x02
#define MPU6000_RA_SELF_TEST_X_ACCEL   0x0D
#define MPU6000_RA_SELF_TEST_Y_ACCEL   0x0E
#define MPU6000_RA_SELF_TEST_Z_ACCEL   0x0F
#define MPU6000_RA_XG_OFFS_USRH        0x13
#define MPU6000_RA_XG_OFFS_USRL        0x14
#define MPU6000_RA_YG_OFFS_USRH        0x15
#define MPU6000_RA_YG_OFFS_USRL        0x16
#define MPU6000_RA_ZG_OFFS_USRH        0x17
#define MPU6000_RA_ZG_OFFS_USRL        0x18
#define MPU6000_RA_SMPLRT_DIV          0x19
#define MPU6000_RA_CONFIG              0x1A
#define MPU6000_RA_GYRO_CONFIG         0x1B
#define MPU6000_RA_ACCEL_CONFIG        0x1C
#define MPU6000_RA_ACCEL_CONFIG_2      0x1D
#define MPU6000_RA_LP_MODE_CTRL        0x1E
#define MPU6000_RA_FIFO_EN             0x23
#define MPU6000_RA_INT_PIN_CFG         0x37
#define MPU6000_RA_INT_ENABLE          0x38
#define MPU6000_RA_INT_STATUS          0x3A
#define MPU6000_RA_ACCEL_XOUT_H        0x3B
#define MPU6000_RA_ACCEL_XOUT_L        0x3C
#define MPU6000_RA_ACCEL_YOUT_H        0x3D
#define MPU6000_RA_ACCEL_YOUT_L        0x3E
#define MPU6000_RA_ACCEL_ZOUT_H        0x3F
#define MPU6000_RA_ACCEL_ZOUT_L        0x40
#define MPU6000_RA_TEMP_OUT_H          0x41
#define MPU6000_RA_TEMP_OUT_L          0x42
#define MPU6000_RA_GYRO_XOUT_H         0x43
#define MPU6000_RA_GYRO_XOUT_L         0x44
#define MPU6000_RA_GYRO_YOUT_H         0x45
#define MPU6000_RA_GYRO_YOUT_L         0x46
#define MPU6000_RA_GYRO_ZOUT_H         0x47
#define MPU6000_RA_GYRO_ZOUT_L         0x48
#define MPU6000_RA_PWR_MGMT_1          0x6B
#define MPU6000_RA_PWR_MGMT_2          0x6C
#define MPU6000_RA_WHO_AM_I            0x75

// ==========================================
// CONFIGURATION VALUES
// ==========================================

// Gyroscope full-scale range
typedef enum {
    GYRO_RANGE_250_DPS  = 0x00,  // ±250°/s
    GYRO_RANGE_500_DPS  = 0x08,  // ±500°/s
    GYRO_RANGE_1000_DPS = 0x10,  // ±1000°/s
    GYRO_RANGE_2000_DPS = 0x18   // ±2000°/s (default for drones)
} GyroRange;

// Accelerometer full-scale range
typedef enum {
    ACCEL_RANGE_2G  = 0x00,  // ±2g
    ACCEL_RANGE_4G  = 0x08,  // ±4g
    ACCEL_RANGE_8G  = 0x10,  // ±8g
    ACCEL_RANGE_16G = 0x18   // ±16g (default for drones)
} AccelRange;

// Digital Low-Pass Filter bandwidth
typedef enum {
    DLPF_256HZ = 0x00,  // 256Hz bandwidth, 8kHz sample rate
    DLPF_188HZ = 0x01,  // 188Hz bandwidth, 1kHz sample rate
    DLPF_98HZ  = 0x02,  // 98Hz bandwidth, 1kHz sample rate
    DLPF_42HZ  = 0x03,  // 42Hz bandwidth, 1kHz sample rate
    DLPF_20HZ  = 0x04,  // 20Hz bandwidth, 1kHz sample rate
    DLPF_10HZ  = 0x05,  // 10Hz bandwidth, 1kHz sample rate
    DLPF_5HZ   = 0x06   // 5Hz bandwidth, 1kHz sample rate
} DLPFBandwidth;

// ==========================================
// DATA STRUCTURES
// ==========================================

// Raw IMU data (direct from sensor registers)
typedef struct {
    int16_t gyro_x;      // Raw gyro X
    int16_t gyro_y;      // Raw gyro Y
    int16_t gyro_z;      // Raw gyro Z
    int16_t accel_x;     // Raw accel X
    int16_t accel_y;     // Raw accel Y
    int16_t accel_z;     // Raw accel Z
    int16_t temp;        // Raw temperature
} MPU6000_RawData;

// Calibrated IMU data (in real units)
typedef struct {
    float gyro_x;        // rad/s
    float gyro_y;        // rad/s
    float gyro_z;        // rad/s
    float accel_x;       // m/s²
    float accel_y;       // m/s²
    float accel_z;       // m/s²
    float temp_c;        // Celsius
} MPU6000_Data;

// IMU calibration offsets
typedef struct {
    float gyro_offset[3];    // X, Y, Z offset in rad/s
    float accel_offset[3];   // X, Y, Z offset in m/s²
    float gyro_scale[3];     // X, Y, Z scale factor
    float accel_scale[3];    // X, Y, Z scale factor
} MPU6000_Calibration;

// ==========================================
// FUNCTION DECLARATIONS
// ==========================================

// Initialize the MPU6000
// Returns true if WHO_AM_I register reads correctly (0x68)
bool mpu6000_init(void);

// Configure gyro range
void mpu6000_set_gyro_range(GyroRange range);

// Configure accelerometer range
void mpu6000_set_accel_range(AccelRange range);

// Configure digital low-pass filter
void mpu6000_set_dlpf(DLPFBandwidth bandwidth);

// Read raw sensor data (14 bytes from ACCEL_XOUT_H to GYRO_ZOUT_L)
void mpu6000_read_raw(MPU6000_RawData *raw);

// Read calibrated sensor data in real units
void mpu6000_read(MPU6000_Data *data);

// Convert raw gyro value to rad/s
float mpu6000_gyro_to_rads(int16_t raw);

// Convert raw accel value to m/s²
float mpu6000_accel_to_ms2(int16_t raw);

// Convert raw temp to Celsius
float mpu6000_temp_to_celsius(int16_t raw);

// Perform gyro calibration (calculate offsets)
void mpu6000_calibrate_gyro(int num_samples);

// Perform accelerometer calibration
void mpu6000_calibrate_accel(int num_samples);

// Check if data is ready (interrupt status)
bool mpu6000_data_ready(void);

// Get WHO_AM_I register value (should be 0x68)
uint8_t mpu6000_who_am_i(void);

// Reset the device
void mpu6000_reset(void);

#endif // MPU6000_H