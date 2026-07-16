#ifndef MAHONY_FILTER_H
#define MAHONY_FILTER_H

#include<stdbool.h>

typedef struct{
    float kp; //propotional gain (how much to trust accelerometer)
    float ki; //Integral gain(correct gyro bias over tine)
    float integral_fb[3]; //integrak feedback for gyro bias (x,y,z)
    float q[4]; //quaternion [w,x,y,z]-current orientation
    float sample_freq; //sample frequency in Hz eg 8000.0f
} MahonyFilter;

//Initialize the mahony filter
//sample_freq: how often mahony_update is called is called (eg 8000 for 8Khz)
void mahony_init(MahonyFilter *filter, float sample_freq);

//update the filter with new IMU data
//gx, gy, gz: Gyroscope reading in rad/s
//ax, ay, az: accelerometer reading in m/s^2 (or g's will be noramlised)
// dt: time step in second (eg 1.0/8000.0=0.000125)
void mahony_update(MahonyFilter *filter, float gx, float gy, float gz, float ax, float ay, float az, float dt);

//Get Euler angles from the current orientation
//return roll, pitch, yaw in radians
void mahony_get_euler(const MahonyFilter *filter, float *roll, float *pitch, float *yaw);

//Get the current quaternion
void mahony_get_quaternion(const MahonyFilter *filter, float q[4]);

//reset the filter to level orientation
void mahony_reset(MahonyFilter *filter);

//set filter gain (for tuning)
void mahony_set_gains(MahonyFilter *filter, float kp, float ki);

#endif //MAHONY_FILTER_H

