#include "mpu6000.h"
#include "hal/hal_spi.h"
#include "system/clock_config.h"
#include <math.h>

// ==========================================
// GLOBAL STATE
// ==========================================

// Current sensor configuration
static GyroRange current_gyro_range = GYRO_RANGE_2000_DPS;
static AccelRange current_accel_range = ACCEL_RANGE_16G;

// Calibration data
static MPU6000_Calibration cal = {
    .gyro_offset = {0.0f, 0.0f, 0.0f},
    .accel_offset = {0.0f, 0.0f, 0.0f},
    .gyro_scale = {1.0f, 1.0f, 1.0f},
    .accel_scale = {1.0f, 1.0f, 1.0f}
};

// SPI bus used for IMU
#define IMU_SPI_BUS SPI_BUS_1

// ==========================================
// CONVERSION FACTORS
// ==========================================

// Gyro sensitivity: LSB per degree/second
static float gyro_sensitivity(void) {
    switch (current_gyro_range) {
        case GYRO_RANGE_250_DPS:  return 131.0f;
        case GYRO_RANGE_500_DPS:  return 65.5f;
        case GYRO_RANGE_1000_DPS: return 32.8f;
        case GYRO_RANGE_2000_DPS: return 16.4f;
        default:                  return 16.4f;
    }
}

// Accel sensitivity: LSB per g
static float accel_sensitivity(void) {
    switch (current_accel_range) {
        case ACCEL_RANGE_2G:  return 16384.0f;
        case ACCEL_RANGE_4G:  return 8192.0f;
        case ACCEL_RANGE_8G:  return 4096.0f;
        case ACCEL_RANGE_16G: return 2048.0f;
        default:              return 2048.0f;
    }
}

// ==========================================
// INITIALIZATION
// ==========================================

bool mpu6000_init(void) {
    
    // Step 1: Reset the device
    mpu6000_reset();
    delay_ms(100);  // Wait for reset to complete
    
    // Step 2: Wake up the device (clear sleep bit)
    spi_write_register(IMU_SPI_BUS, MPU6000_RA_PWR_MGMT_1, 0x00);
    delay_ms(10);
    
    // Step 3: Verify WHO_AM_I register
    uint8_t who_am_i = mpu6000_who_am_i();
    if (who_am_i != 0x68) {
        return false;  // Device not found or wrong chip
    }
    
    // Step 4: Configure gyro range (±2000°/s)
    mpu6000_set_gyro_range(GYRO_RANGE_2000_DPS);
    
    // Step 5: Configure accelerometer range (±16g)
    mpu6000_set_accel_range(ACCEL_RANGE_16G);
    
    // Step 6: Configure DLPF (256Hz for 8kHz sampling)
    mpu6000_set_dlpf(DLPF_256HZ);
    
    // Step 7: Configure sample rate divider
    // Sample rate = gyro_output_rate / (1 + SMPLRT_DIV)
    // For 8kHz: 8000 = 8000 / (1 + 0), so SMPLRT_DIV = 0
    spi_write_register(IMU_SPI_BUS, MPU6000_RA_SMPLRT_DIV, 0x00);
    
    // Step 8: Configure interrupt
    // Enable data ready interrupt on INT pin
    spi_write_register(IMU_SPI_BUS, MPU6000_RA_INT_PIN_CFG, 0x00);
    spi_write_register(IMU_SPI_BUS, MPU6000_RA_INT_ENABLE, 0x01);
    
    return true;
}

void mpu6000_reset(void) {
    // Set DEVICE_RESET bit in PWR_MGMT_1
    spi_write_register(IMU_SPI_BUS, MPU6000_RA_PWR_MGMT_1, 0x80);
}

uint8_t mpu6000_who_am_i(void) {
    uint8_t data;
    spi_read_register(IMU_SPI_BUS, MPU6000_RA_WHO_AM_I, &data, 1);
    return data;
}

// ==========================================
// CONFIGURATION
// ==========================================

void mpu6000_set_gyro_range(GyroRange range) {
    spi_write_register(IMU_SPI_BUS, MPU6000_RA_GYRO_CONFIG, range);
    current_gyro_range = range;
}

void mpu6000_set_accel_range(AccelRange range) {
    spi_write_register(IMU_SPI_BUS, MPU6000_RA_ACCEL_CONFIG, range);
    current_accel_range = range;
}

void mpu6000_set_dlpf(DLPFBandwidth bandwidth) {
    spi_write_register(IMU_SPI_BUS, MPU6000_RA_CONFIG, bandwidth);
}

// ==========================================
// DATA READING
// ==========================================

void mpu6000_read_raw(MPU6000_RawData *raw) {
    uint8_t buffer[14];
    
    // Read 14 bytes starting from ACCEL_XOUT_H
    spi_read_register(IMU_SPI_BUS, MPU6000_RA_ACCEL_XOUT_H, buffer, 14);
    
    // Parse the buffer into raw values
    raw->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    raw->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    raw->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    raw->temp    = (int16_t)((buffer[6] << 8) | buffer[7]);
    raw->gyro_x  = (int16_t)((buffer[8] << 8) | buffer[9]);
    raw->gyro_y  = (int16_t)((buffer[10] << 8) | buffer[11]);
    raw->gyro_z  = (int16_t)((buffer[12] << 8) | buffer[13]);
}

void mpu6000_read(MPU6000_Data *data) {
    MPU6000_RawData raw;
    mpu6000_read_raw(&raw);
    
    // Convert to real units with calibration
    data->gyro_x  = mpu6000_gyro_to_rads(raw.gyro_x)  - cal.gyro_offset[0];
    data->gyro_y  = mpu6000_gyro_to_rads(raw.gyro_y)  - cal.gyro_offset[1];
    data->gyro_z  = mpu6000_gyro_to_rads(raw.gyro_z)  - cal.gyro_offset[2];
    data->accel_x = mpu6000_accel_to_ms2(raw.accel_x) - cal.accel_offset[0];
    data->accel_y = mpu6000_accel_to_ms2(raw.accel_y) - cal.accel_offset[1];
    data->accel_z = mpu6000_accel_to_ms2(raw.accel_z) - cal.accel_offset[2];
    data->temp_c  = mpu6000_temp_to_celsius(raw.temp);
}

// ==========================================
// CONVERSION FUNCTIONS
// ==========================================

float mpu6000_gyro_to_rads(int16_t raw) {
    // Convert raw to degrees/second, then to radians/second
    float deg_per_sec = (float)raw / gyro_sensitivity();
    return deg_per_sec * (M_PI / 180.0f);
}

float mpu6000_accel_to_ms2(int16_t raw) {
    // Convert raw to g, then to m/s²
    float g = (float)raw / accel_sensitivity();
    return g * 9.80665f;
}

float mpu6000_temp_to_celsius(int16_t raw) {
    // Temperature in Celsius = (raw / 340.0) + 36.53
    return ((float)raw / 340.0f) + 36.53f;
}

// ==========================================
// CALIBRATION
// ==========================================

void mpu6000_calibrate_gyro(int num_samples) {
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    
    // Collect samples while stationary
    for (int i = 0; i < num_samples; i++) {
        MPU6000_RawData raw;
        mpu6000_read_raw(&raw);
        
        sum_x += mpu6000_gyro_to_rads(raw.gyro_x);
        sum_y += mpu6000_gyro_to_rads(raw.gyro_y);
        sum_z += mpu6000_gyro_to_rads(raw.gyro_z);
        
        delay_ms(1);  // ~1kHz sampling during calibration
    }
    
    // Calculate average (this is the bias)
    cal.gyro_offset[0] = sum_x / (float)num_samples;
    cal.gyro_offset[1] = sum_y / (float)num_samples;
    cal.gyro_offset[2] = sum_z / (float)num_samples;
}

void mpu6000_calibrate_accel(int num_samples) {
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    
    for (int i = 0; i < num_samples; i++) {
        MPU6000_RawData raw;
        mpu6000_read_raw(&raw);
        
        sum_x += mpu6000_accel_to_ms2(raw.accel_x);
        sum_y += mpu6000_accel_to_ms2(raw.accel_y);
        sum_z += mpu6000_accel_to_ms2(raw.accel_z);
        
        delay_ms(1);
    }
    
    // Accelerometer should read (0, 0, g) when level
    cal.accel_offset[0] = sum_x / (float)num_samples;
    cal.accel_offset[1] = sum_y / (float)num_samples;
    cal.accel_offset[2] = (sum_z / (float)num_samples) - 9.80665f;
}

// ==========================================
// STATUS
// ==========================================

bool mpu6000_data_ready(void) {
    uint8_t status;
    spi_read_register(IMU_SPI_BUS, MPU6000_RA_INT_STATUS, &status, 1);
    return (status & 0x01) ? true : false;  // Bit 0 = data ready
}