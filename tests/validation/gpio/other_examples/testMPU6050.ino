/*
 * MPU6050 Sensor Unity Test
 *
 * This test demonstrates the Wokwi automation infrastructure for MPU6050 sensor testing
 * using Unity test framework. It shows how to:
 * - Set sensor values using wokwi_mpu6050_* functions
 * - Read values back via I2C to verify functionality
 * - Test accelerometer, gyroscope, and temperature readings
 * - Use Unity assertions for proper test validation
 *
 * Circuit Requirements:
 * - MPU6050 sensor connected via I2C (SDA=21, SCL=22 by default)
 * - Component ID: "imu1" in diagram.json
 *
 * Expected behavior:
 * 1. Initialize I2C and MPU6050
 * 2. Set known test values via automation
 * 3. Read back values via I2C
 * 4. Verify readings match expected values using Unity assertions
 */

#include <Arduino.h>
#include <unity.h>
#include <Wire.h>
#include "wokwi.h"

// MPU6050 I2C address and register definitions
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define TEMP_OUT_H   0x41
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47

// Test parameters
const char* MPU_PART_ID = "imu1";
const float TEST_ACCEL_X = 1.5f;
const float TEST_ACCEL_Y = -0.8f;
const float TEST_ACCEL_Z = 0.2f;
const float TEST_GYRO_X = 45.0f;
const float TEST_GYRO_Y = -30.0f;
const float TEST_GYRO_Z = 15.0f;
const float TEST_TEMP = 25.5f;

// Tolerance for floating point comparisons
const float TOLERANCE = 0.1f;

// Function declarations for Unity tests
void test_basic_communication();
void test_accelerometer_values();
void test_gyroscope_values();
void test_temperature_value();
void test_batch_operations();
void test_motion_simulation();

// Helper function declaration
int16_t readRegister16(uint8_t reg);

/**
 * Test basic I2C communication with MPU6050
 */
void test_basic_communication() {
  // Read WHO_AM_I register (should return 0x68)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x75); // WHO_AM_I register
  Wire.endTransmission();

  Wire.requestFrom(MPU6050_ADDR, 1);
  TEST_ASSERT_TRUE(Wire.available());

  uint8_t whoAmI = Wire.read();
  TEST_ASSERT_EQUAL_HEX8(0x68, whoAmI);
}

/**
 * Test accelerometer value setting and reading
 */
void test_accelerometer_values() {
  // Set test acceleration values via automation
  wokwi_mpu6050_set_acceleration(MPU_PART_ID, TEST_ACCEL_X, TEST_ACCEL_Y, TEST_ACCEL_Z);
  wokwi_delay(50); // Allow values to propagate

  // Read accelerometer values via I2C
  int16_t accelX = readRegister16(ACCEL_XOUT_H);
  int16_t accelY = readRegister16(ACCEL_YOUT_H);
  int16_t accelZ = readRegister16(ACCEL_ZOUT_H);

  // Convert raw values to g-force (assuming ±2g scale, 16384 LSB/g)
  float accelX_g = accelX / 16384.0f;
  float accelY_g = accelY / 16384.0f;
  float accelZ_g = accelZ / 16384.0f;

  // Verify values are within tolerance using Unity assertions
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, TEST_ACCEL_X, accelX_g);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, TEST_ACCEL_Y, accelY_g);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, TEST_ACCEL_Z, accelZ_g);
}

/**
 * Test gyroscope value setting and reading
 */
void test_gyroscope_values() {
  // Set test rotation values via automation
  wokwi_mpu6050_set_rotation(MPU_PART_ID, TEST_GYRO_X, TEST_GYRO_Y, TEST_GYRO_Z);
  wokwi_delay(50);

  // Read gyroscope values via I2C
  int16_t gyroX = readRegister16(GYRO_XOUT_H);
  int16_t gyroY = readRegister16(GYRO_YOUT_H);
  int16_t gyroZ = readRegister16(GYRO_ZOUT_H);

  // Convert raw values to degrees/second (assuming ±250°/s scale, 131 LSB/°/s)
  float gyroX_dps = gyroX / 131.0f;
  float gyroY_dps = gyroY / 131.0f;
  float gyroZ_dps = gyroZ / 131.0f;

  // Verify values are within tolerance using Unity assertions
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, TEST_GYRO_X, gyroX_dps);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, TEST_GYRO_Y, gyroY_dps);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, TEST_GYRO_Z, gyroZ_dps);
}

/**
 * Test temperature value setting and reading
 */
void test_temperature_value() {
  // Set test temperature via automation
  wokwi_mpu6050_set_temperature(MPU_PART_ID, TEST_TEMP);
  wokwi_delay(50);

  // Read temperature via I2C
  int16_t tempRaw = readRegister16(TEMP_OUT_H);

  // Convert to Celsius (formula from MPU6050 datasheet)
  float temperature = (tempRaw / 340.0f) + 36.53f;

  // Verify temperature is within tolerance using Unity assertion
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, TEST_TEMP, temperature);
}

/**
 * Test batch operations and get functions
 */
void test_batch_operations() {
  // Set all values at once
  wokwi_mpu6050_set_all(MPU_PART_ID,
                        2.0f, -1.0f, 0.5f,    // acceleration
                        90.0f, -45.0f, 30.0f, // rotation
                        26.0f);                // temperature
  wokwi_delay(100);

  // Verify using get functions
  float accelX = wokwi_mpu6050_get_accel(MPU_PART_ID, "X");
  float accelY = wokwi_mpu6050_get_accel(MPU_PART_ID, "Y");
  float accelZ = wokwi_mpu6050_get_accel(MPU_PART_ID, "Z");
  float gyroX = wokwi_mpu6050_get_rotation(MPU_PART_ID, "X");
  float gyroY = wokwi_mpu6050_get_rotation(MPU_PART_ID, "Y");
  float gyroZ = wokwi_mpu6050_get_rotation(MPU_PART_ID, "Z");
  float temp = wokwi_mpu6050_get_temperature(MPU_PART_ID);

  // Check if values match what we set (automation get functions return set values)
  TEST_ASSERT_EQUAL_FLOAT(2.0f, accelX);
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, accelY);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, accelZ);
  TEST_ASSERT_EQUAL_FLOAT(90.0f, gyroX);
  TEST_ASSERT_EQUAL_FLOAT(-45.0f, gyroY);
  TEST_ASSERT_EQUAL_FLOAT(30.0f, gyroZ);
  TEST_ASSERT_EQUAL_FLOAT(26.0f, temp);
}

/**
 * Test motion simulation patterns
 */
void test_motion_simulation() {
  // Test rest position
  wokwi_mpu6050_simulate_motion(MPU_PART_ID, "rest");
  wokwi_delay(100);

  float restAccelZ = wokwi_mpu6050_get_accel(MPU_PART_ID, "Z");
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 1.0f, restAccelZ);

  // Test tilt motion
  wokwi_mpu6050_simulate_motion(MPU_PART_ID, "tilt");
  wokwi_delay(100);

  float tiltAccelX = wokwi_mpu6050_get_accel(MPU_PART_ID, "X");
  float tiltRotZ = wokwi_mpu6050_get_rotation(MPU_PART_ID, "Z");

  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.7f, tiltAccelX);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 45.0f, tiltRotZ);

  // Test shake motion (just verify it completes without error)
  wokwi_mpu6050_simulate_motion(MPU_PART_ID, "shake");
  wokwi_delay(300); // Give shake sequence time to complete

  // If we reach here, shake simulation completed successfully
  TEST_ASSERT_TRUE(true);
}

/**
 * Helper function to read 16-bit register value
 */
int16_t readRegister16(uint8_t reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(MPU6050_ADDR, 2);
  if (Wire.available() >= 2) {
    uint8_t high = Wire.read();
    uint8_t low = Wire.read();
    return (high << 8) | low;
  }
  return 0;
}

void setUp(void) {
  // Reset any state before each test
  // Initialize I2C if not already done
  Wire.begin();

  // Initialize MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0); // Wake up MPU6050
  Wire.endTransmission();

  // Short delay for sensor stabilization
  delay(50);
}

void tearDown(void) {
  // Clean up after each test if needed
}

void setup() {
  UNITY_BEGIN();

  // Run Unity tests
  RUN_TEST(test_basic_communication);
  RUN_TEST(test_accelerometer_values);
  RUN_TEST(test_gyroscope_values);
  RUN_TEST(test_temperature_value);
  RUN_TEST(test_batch_operations);
  RUN_TEST(test_motion_simulation);

  UNITY_END();
}

void loop() {
  // Tests completed in setup()
}
