#include "mahony_filter.h"
#include <math.h>
#include <stddef.h>

static float inv_sqrt(float x){
    return 1.0f/sqrt(x);
}

void mahony_init(MahonyFilter *filter, float sample_freq){
    //default gains (work well for most setup)
    filter->kp=0.5f;   //proportional gain
    filter->ki=0.1f;   //integral gain

    //clear intergral feedback 
    filter->integral_fb[0]=0.0f;
    filter->integral_fb[1]=0.0f;
    filter->integral_fb[2]=0.0f;

    //start with level orintation (identity quaternion)
    filter->q[0]=1.0f; //w
    filter->q[1]=0.0;  //x
    filter->q[2]=0.0f; //y
    filter->q[3]=0.0f; //z

    filter->sample_freq=sample_freq;
}

void mahony_update(MahonyFilter *filter, float gx, float gy, float gz, float ax, float ay, float az, float dt){
    float q0=filter->q[0]; //w
    float q1=filter->q[1]; //x
    float q2=filter->q[2]; //y
    float q3=filter->q[3]; //z

    float recip_norm;
    float half_vx, half_vy, half_vz;
    float half_ex, half_ey, half_ez;
    float qa, qb, qc;

    // STEP 1: Normalize accelerometer reading 
    //only trust accelerometer if its magnitude is reasonable
    // (not free-failing, not experiencing huge acceleration)
    recip_norm = inv_sqrt(ax*ax + ay*ay + az*az);
    
    //If magnitude is too far from 1g, skip accelerometer correction
    float accel_mag=1.0f/recip_norm;
    bool use_accel=(accel_mag>0.5f && accel_mag<2.0f);

    if (use_accel){
        ax *=recip_norm;
        ay *=recip_norm;
        az *=recip_norm;
    }
    // STEP 2: Calculate error from accelerometer
    // The accelerometer measures gravity direction
    // Compare with the gravity direction predicted by out quaternion
    if (use_accel){
        //estimated gravity vector form current quaternion
        //Gravity in world frame (0,0,1) poiting down
        // Rotated to body frame: 
        half_vx = q1 * q3 - q0 * q2; // 2*(q1*q3 - q0*q2)
        half_vy = q0 * q1 + q2 * q3; //// 2*(q0*q1 + q2*q3)
        half_vz = q0 * q0 - 0.5f + q3 * q3; // q0² - q1² - q2² + q3² approx

        // Actually these should be:
        // vx = 2*(q1*q3 - q0*q2)
        // vy = 2*(q0*q1 + q2*q3)
        // vz = q0² - q1² - q2² + q3²
        
        // Simplified half values (error is same direction):
        half_vx = q1 * q3 - q0 * q2;
        half_vy = q0 * q1 + q2 * q3;
        half_vz = q0 * q0 - 0.5f + q3 * q3;
        
        // Error = cross product of measured gravity and estimated gravity
        half_ex = (ay * half_vz - az * half_vy);
        half_ey = (az * half_vx - ax * half_vz);
        half_ez = (ax * half_vy - ay * half_vx);

        //Apply integral feedback
        // Slowly corrects gyroscope bias

        if (filter->ki > 0.0f){
            filter->integral_fb[0] += filter->ki * half_ex * dt;
            filter->integral_fb[1] += filter->ki * half_ey * dt;
            filter->integral_fb[2] += filter->ki * half_ez * dt;

            //apply integral correction to gyro
            gx += filter->integral_fb[0];
            gy += filter->integral_fb[1];
            gz += filter->integral_fb[2];
        }

        // Step 4: Apply proportional correction 
        // immediately corrects based on current error
        gx += filter->kp * half_ex;
        gy += filter->kp * half_ey;
        gz += filter->kp * half_ez;
    }

    // STEP 5: Integrate quaternion using corrected gyro
    // q_dot = 0.5 * q x w
    // q_new = q+ q_dot * dt

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    qa=q0;
    qb=q1;
    qc=q2;

    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += ( qa * gx + qc * gz - q3 * gy);
    q2 += ( qa * gy - qb * gz + q3 * gx);
    q3 += ( qa * gz + qb * gy - qc * gx);

    //STEP 6: Normalize quaternion
    recip_norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    filter->q[0] = q0 * recip_norm;
    filter->q[1] = q1 * recip_norm;
    filter->q[2] = q2 * recip_norm;
    filter->q[3] = q3 * recip_norm;
}

void mahony_get_euler(const MahonyFilter *filter, float *roll, float *pitch, float *yaw){
    float q0= filter->q[0]; //w
    float q1= filter->q[1]; //x
    float q2= filter->q[2]; //y
    float q3= filter->q[3]; //z
    
    //roll rotation about x-axis
    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));

    //Pitch: rotation about Y axis
    float sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
    if (sin_pitch > 1.0f) sin_pitch = 1.0f;
    if (sin_pitch < -1.0f) sin_pitch = -1.0f;
    *pitch = asinf(sin_pitch);

    //Yaw: rotation about Z axis
    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

void mahony_get_quaternion(const MahonyFilter *filter, float q[4]) {
    q[0] = filter->q[0];
    q[1] = filter->q[1];
    q[2] = filter->q[2];
    q[3] = filter->q[3];
}

void mahony_reset(MahonyFilter *filter) {
    filter->q[0] = 1.0f;
    filter->q[1] = 0.0f;
    filter->q[2] = 0.0f;
    filter->q[3] = 0.0f;
    
    filter->integral_fb[0] = 0.0f;
    filter->integral_fb[1] = 0.0f;
    filter->integral_fb[2] = 0.0f;
}

void mahony_set_gains(MahonyFilter *filter, float kp, float ki) {
    filter->kp = kp;
    filter->ki = ki;
}