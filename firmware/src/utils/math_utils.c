#include "math_utils.h"
#include <math.h>

// ============================================================
// VECTOR OPERATIONS
// ============================================================

void vector_cross(const Vector3 *a, const Vector3 *b, Vector3 *result) {
    // Cross product formula:
    // result.x = a.y*b.z - a.z*b.y
    // result.y = a.z*b.x - a.x*b.z
    // result.z = a.x*b.y - a.y*b.x
    
    result->x = a->y * b->z - a->z * b->y;
    result->y = a->z * b->x - a->x * b->z;
    result->z = a->x * b->y - a->y * b->x;
}

float vector_dot(const Vector3 *a, const Vector3 *b) {
    // Dot product: sum of component-wise products
    // a·b = ax*bx + ay*by + az*bz
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

void vector_normalize(Vector3 *v) {
    // 1. Calculate magnitude: sqrt(x² + y² + z²)
    float mag = vector_magnitude(v);
    
    // 2. Guard against division by zero
    // If magnitude is zero, vector has no direction to normalize
    if (mag > 0.000001f) {
        // 3. Divide each component by magnitude
        float inv_mag = 1.0f / mag;
        v->x *= inv_mag;
        v->y *= inv_mag;
        v->z *= inv_mag;
    }
}

void vector_add(const Vector3 *a, const Vector3 *b, Vector3 *result) {
    result->x = a->x + b->x;
    result->y = a->y + b->y;
    result->z = a->z + b->z;
}

void vector_sub(const Vector3 *a, const Vector3 *b, Vector3 *result) {
    result->x = a->x - b->x;
    result->y = a->y - b->y;
    result->z = a->z - b->z;
}

void vector_scale(const Vector3 *v, float scalar, Vector3 *result) {
    result->x = v->x * scalar;
    result->y = v->y * scalar;
    result->z = v->z * scalar;
}

float vector_magnitude(const Vector3 *v) {
    // sqrt(x² + y² + z²)
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}


// ============================================================
// QUATERNION OPERATIONS
// ============================================================

void quaternion_multiply(const Quaternion *a, const Quaternion *b, Quaternion *result) {
    // Hamilton product: q_result = a ⊗ b
    // 
    // Scalar part: aw*bw - ax*bx - ay*by - az*bz
    // Vector part: aw*(bx,by,bz) + bw*(ax,ay,az) + (ax,ay,az) × (bx,by,bz)
    //
    // Expanded to components:
    // w = aw*bw - ax*bx - ay*by - az*bz
    // x = aw*bx + ax*bw + ay*bz - az*by
    // y = aw*by - ax*bz + ay*bw + az*bx
    // z = aw*bz + ax*by - ay*bx + az*bw
    
    result->w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
    result->x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
    result->y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
    result->z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;
}

void quaternion_normalize(Quaternion *q) {
    // Same principle as vector normalization
    // Magnitude = sqrt(w² + x² + y² + z²)
    float mag = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    
    if (mag > 0.000001f) {
        float inv_mag = 1.0f / mag;
        q->w *= inv_mag;
        q->x *= inv_mag;
        q->y *= inv_mag;
        q->z *= inv_mag;
    }
}

void quaternion_to_euler(const Quaternion *q, float *roll, float *pitch, float *yaw) {
    // Convert unit quaternion to Tait-Bryan angles (ZYX convention)
    //
    // Roll (φ): rotation about X axis
    // Pitch (θ): rotation about Y axis
    // Yaw (ψ): rotation about Z axis
    //
    // Formulas from rotation matrix elements:
    
    
    // R31 = 2*(w*y + z*x)
    float r31 = 2.0f * (q->w * q->y + q->z * q->x);
    
    // R32 = 2*(w*z - x*y)
    float r32 = 2.0f * (q->w * q->z - q->x * q->y);
    
    // R33 = w² - x² - y² + z²
    float r33 = q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z;
    
    // R21 = 2*(w*x + y*z)
    float r21 = 2.0f * (q->w * q->x + q->y * q->z);
    
    // R11 = w² + x² - y² - z²
    float r11 = q->w * q->w + q->x * q->x - q->y * q->y - q->z * q->z;
    
    // Pitch: asin(-r31), clamped to [-1, 1] for numerical stability
    float r31_clamped = r31;
    if (r31_clamped > 1.0f) r31_clamped = 1.0f;
    if (r31_clamped < -1.0f) r31_clamped = -1.0f;
    *pitch = asinf(-r31_clamped);
    
    // Roll: atan2(r32, r33)
    *roll = atan2f(r32, r33);
    
    // Yaw: atan2(r21, r11)
    *yaw = atan2f(r21, r11);
}

void euler_to_quaternion(float roll, float pitch, float yaw, Quaternion *q) {
    // Convert Euler angles to quaternion
    // Rotation order: Z (yaw) → Y (pitch) → X (roll)
    //
    // Each rotation is represented as:
    // q_yaw   = (cos(yaw/2),   0, 0, sin(yaw/2))
    // q_pitch = (cos(pitch/2), 0, sin(pitch/2), 0)
    // q_roll  = (cos(roll/2),  sin(roll/2), 0, 0)
    //
    // Combined: q = q_yaw * q_pitch * q_roll
    
    float half_roll  = roll * 0.5f;
    float half_pitch = pitch * 0.5f;
    float half_yaw   = yaw * 0.5f;
    
    float cr = cosf(half_roll);
    float sr = sinf(half_roll);
    float cp = cosf(half_pitch);
    float sp = sinf(half_pitch);
    float cy = cosf(half_yaw);
    float sy = sinf(half_yaw);
    
    // Multiply the three quaternions:
    // q = q_yaw ⊗ q_pitch ⊗ q_roll
    q->w = cr * cp * cy + sr * sp * sy;
    q->x = sr * cp * cy - cr * sp * sy;
    q->y = cr * sp * cy + sr * cp * sy;
    q->z = cr * cp * sy - sr * sp * cy;
}

void quaternion_conjugate(const Quaternion *q, Quaternion *result) {
    // Conjugate: negate the vector part, keep scalar part
    // For unit quaternions: conjugate = inverse rotation
    // q * conjugate(q) = (1, 0, 0, 0) = identity rotation
    result->w =  q->w;
    result->x = -q->x;
    result->y = -q->y;
    result->z = -q->z;
}

void quaternion_rotate_vector(const Quaternion *q, const Vector3 *v, Vector3 *result) {
    // Rotate vector v by quaternion q
    // Formula: v' = q ⊗ v ⊗ q*
    // Where v is treated as pure quaternion (0, vx, vy, vz)
    // and q* is the conjugate of q
    
    // Create pure quaternion from vector
    Quaternion vq;
    vq.w = 0.0f;
    vq.x = v->x;
    vq.y = v->y;
    vq.z = v->z;
    
    // Get conjugate of q
    Quaternion q_conj;
    quaternion_conjugate(q, &q_conj);
    
    // v' = q ⊗ v ⊗ q*
    Quaternion temp;
    quaternion_multiply(q, &vq, &temp);
    
    Quaternion rotated;
    quaternion_multiply(&temp, &q_conj, &rotated);
    
    // Extract vector from result quaternion
    result->x = rotated.x;
    result->y = rotated.y;
    result->z = rotated.z;
}

void quaternion_identity(Quaternion *q) {
    // Identity quaternion: represents zero rotation
    // w=1 means cos(0/2) = 1, so rotation angle = 0
    q->w = 1.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
}


// ============================================================
// ANGLE CONVERSIONS
// ============================================================

float deg_to_rad(float degrees) {
    return degrees * (M_PI / 180.0f);
}

float rad_to_deg(float radians) {
    return radians * (180.0f / M_PI);
}

float angle_normalize(float angle_rad) {
    // Wrap angle to [-π, π]
    // Example: 370° → 10°, -190° → 170°
    
    while (angle_rad > M_PI) {
        angle_rad -= 2.0f * M_PI;
    }
    while (angle_rad < -M_PI) {
        angle_rad += 2.0f * M_PI;
    }
    return angle_rad;
}