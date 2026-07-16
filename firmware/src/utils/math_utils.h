#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <math.h>
#include <stdint.h>

// ============================================================
// DATA TYPES
// ============================================================

// A quaternion represents 3D orientation
// w = cos(half_angle), the scalar "real" part
// (x,y,z) = axis * sin(half_angle), the vector "imaginary" part
// Unit quaternion: w² + x² + y² + z² = 1
typedef struct {
    float w;    // Scalar component (cosine of half the rotation angle)
    float x;    // X component of the rotation axis
    float y;    // Y component of the rotation axis
    float z;    // Z component of the rotation axis
} Quaternion;

// A 3D vector for directions, positions, rates, forces
typedef struct {
    float x;    // X component
    float y;    // Y component
    float z;    // Z component
} Vector3;

// Euler angles in radians
// Roll:  rotation around X axis (forward axis)
// Pitch: rotation around Y axis (right axis)  
// Yaw:   rotation around Z axis (down axis)
typedef struct {
    float roll;     // Radians, positive = right side down
    float pitch;    // Radians, positive = nose up
    float yaw;      // Radians, positive = nose right
} EulerAngles;


// ============================================================
// VECTOR OPERATIONS
// ============================================================

// Cross product: a × b = perpendicular vector to both a and b
// |a × b| = |a||b|sin(θ), direction by right-hand rule
// Used in Mahony filter to find orientation error direction
void vector_cross(const Vector3 *a, const Vector3 *b, Vector3 *result);

// Dot product: a · b = |a||b|cos(θ)
// Returns scalar. Positive if vectors point same direction
// Used to find angle between vectors, projection length
float vector_dot(const Vector3 *a, const Vector3 *b);

// Scale vector to magnitude 1.0 while preserving direction
// Critical after many operations to prevent drift
void vector_normalize(Vector3 *v);

// result = a + b
void vector_add(const Vector3 *a, const Vector3 *b, Vector3 *result);

// result = a - b
void vector_sub(const Vector3 *a, const Vector3 *b, Vector3 *result);

// result = v * s  (multiply each component by scalar)
void vector_scale(const Vector3 *v, float scalar, Vector3 *result);

// Calculate magnitude (length) of vector
float vector_magnitude(const Vector3 *v);


// ============================================================
// QUATERNION OPERATIONS
// ============================================================

// Hamilton product: combines two rotations
// q_result = q1 * q2 means "apply q2, then apply q1"
// NOT commutative: q1 * q2 ≠ q2 * q1
void quaternion_multiply(const Quaternion *a, const Quaternion *b, Quaternion *result);

// Scale quaternion to unit magnitude
// Must call periodically to prevent drift from floating point errors
void quaternion_normalize(Quaternion *q);

// Convert quaternion to Euler angles (roll, pitch, yaw) in radians
// Pitch is limited to ±90° to avoid gimbal lock ambiguity
void quaternion_to_euler(const Quaternion *q, float *roll, float *pitch, float *yaw);

// Create quaternion from Euler angles
// Rotation order: Z (yaw) → Y (pitch) → X (roll)
void euler_to_quaternion(float roll, float pitch, float yaw, Quaternion *q);

// Conjugate: reverse the rotation
// For unit quaternion: conjugate = inverse
// q * conjugate(q) = identity rotation
void quaternion_conjugate(const Quaternion *q, Quaternion *result);

// Rotate a 3D vector by a quaternion: v' = q * v * conjugate(q)
// Used to transform gravity vector from world frame to body frame
void quaternion_rotate_vector(const Quaternion *q, const Vector3 *v, Vector3 *result);

// Create identity quaternion (represents zero rotation)
// w=1, x=0, y=0, z=0
void quaternion_identity(Quaternion *q);


// ============================================================
// ANGLE CONVERSIONS
// ============================================================

// Degrees to radians
// radians = degrees × π / 180
float deg_to_rad(float degrees);

// Radians to degrees
// degrees = radians × 180 / π
float rad_to_deg(float radians);

// Wrap angle to range [-π, π] or [-180°, 180°]
// Prevents angle values from growing unbounded during continuous yaw rotation
float angle_normalize(float angle_rad);

#endif // MATH_UTILS_H