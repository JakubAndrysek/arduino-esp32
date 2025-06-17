/*
 * WOKWI AUTOMATION LIBRARY - MPU HELPER FUNCTIONS
 */

#include <unity.h>
#include <Arduino.h>
#include <stdarg.h>
#include "wokwi.h"


// =============================================================================
// MPU6050 SENSOR CONTROL FUNCTIONS
// =============================================================================

/**
 * Set MPU6050 X-axis acceleration value (g-force units)
 * @param partId - Component ID (e.g., "imu1")
 * @param accelX - Acceleration value in g (1g = 9.80665 m/s²)
 */
void wokwi_mpu6050_set_accel_x(const char *partId, float accelX) {
  wokwi_set_control_float(partId, "accelX", accelX);
}

/**
 * Set MPU6050 Y-axis acceleration value (g-force units)
 */
void wokwi_mpu6050_set_accel_y(const char *partId, float accelY) {
  wokwi_set_control_float(partId, "accelY", accelY);
}

/**
 * Set MPU6050 Z-axis acceleration value (g-force units)
 */
void wokwi_mpu6050_set_accel_z(const char *partId, float accelZ) {
  wokwi_set_control_float(partId, "accelZ", accelZ);
}

/**
 * Set MPU6050 acceleration for all three axes at once
 */
void wokwi_mpu6050_set_acceleration(const char *partId, float accelX, float accelY, float accelZ) {
  wokwi_mpu6050_set_accel_x(partId, accelX);
  wokwi_mpu6050_set_accel_y(partId, accelY);
  wokwi_mpu6050_set_accel_z(partId, accelZ);
}

/**
 * Set MPU6050 X-axis rotation value (degrees per second)
 */
void wokwi_mpu6050_set_rotation_x(const char *partId, float rotationX) {
  wokwi_set_control_float(partId, "rotationX", rotationX);
}

/**
 * Set MPU6050 Y-axis rotation value (degrees per second)
 */
void wokwi_mpu6050_set_rotation_y(const char *partId, float rotationY) {
  wokwi_set_control_float(partId, "rotationY", rotationY);
}

/**
 * Set MPU6050 Z-axis rotation value (degrees per second)
 */
void wokwi_mpu6050_set_rotation_z(const char *partId, float rotationZ) {
  wokwi_set_control_float(partId, "rotationZ", rotationZ);
}

/**
 * Set MPU6050 rotation for all three axes at once
 */
void wokwi_mpu6050_set_rotation(const char *partId, float rotationX, float rotationY, float rotationZ) {
  wokwi_mpu6050_set_rotation_x(partId, rotationX);
  wokwi_mpu6050_set_rotation_y(partId, rotationY);
  wokwi_mpu6050_set_rotation_z(partId, rotationZ);
}

/**
 * Set MPU6050 temperature value (Celsius)
 */
void wokwi_mpu6050_set_temperature(const char *partId, float temperature) {
  wokwi_set_control_float(partId, "temperature", temperature);
}

/**
 * Set all MPU6050 sensor values at once (comprehensive setup)
 */
void wokwi_mpu6050_set_all(const char *partId,
                           float accelX, float accelY, float accelZ,
                           float rotationX, float rotationY, float rotationZ,
                           float temperature) {
  wokwi_mpu6050_set_acceleration(partId, accelX, accelY, accelZ);
  wokwi_mpu6050_set_rotation(partId, rotationX, rotationY, rotationZ);
  wokwi_mpu6050_set_temperature(partId, temperature);
}

/**
 * Get MPU6050 acceleration value for specified axis
 */
double wokwi_mpu6050_get_accel(const char *partId, const char *axis) {
  char control[16];
  snprintf(control, sizeof(control), "accel%s", axis);
  return wokwi_get_control(partId, control);
}

/**
 * Get MPU6050 rotation value for specified axis
 */
double wokwi_mpu6050_get_rotation(const char *partId, const char *axis) {
  char control[16];
  snprintf(control, sizeof(control), "rotation%s", axis);
  return wokwi_get_control(partId, control);
}

/**
 * Get MPU6050 temperature value
 */
double wokwi_mpu6050_get_temperature(const char *partId) {
  return wokwi_get_control(partId, "temperature");
}

/**
 * Simulate motion by setting realistic movement patterns
 * Useful for testing motion detection algorithms
 */
void wokwi_mpu6050_simulate_motion(const char *partId, const char *motionType) {
  if (strcmp(motionType, "shake") == 0) {
    // Simulate shaking motion
    wokwi_mpu6050_set_acceleration(partId, 2.0, 1.5, 0.8);
    wokwi_delay(100);
    wokwi_mpu6050_set_acceleration(partId, -1.8, -1.2, 1.2);
    wokwi_delay(100);
    wokwi_mpu6050_set_acceleration(partId, 0.0, 0.0, 1.0); // Return to rest
  } else if (strcmp(motionType, "tilt") == 0) {
    // Simulate tilting motion
    wokwi_mpu6050_set_acceleration(partId, 0.7, 0.0, 0.7);
    wokwi_mpu6050_set_rotation(partId, 0.0, 0.0, 45.0);
  } else if (strcmp(motionType, "rest") == 0) {
    // Return to rest position (default orientation)
    wokwi_mpu6050_set_acceleration(partId, 0.0, 0.0, 1.0);
    wokwi_mpu6050_set_rotation(partId, 0.0, 0.0, 0.0);
    wokwi_mpu6050_set_temperature(partId, 24.0);
  }
}
