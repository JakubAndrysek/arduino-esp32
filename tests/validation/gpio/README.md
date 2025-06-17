# WOKWI AUTOMATION LIBRARY - ESP32 GPIO TESTING

This library provides a command-response pattern for automated testing of ESP32
functionality within the Wokwi simulator environment.

ARCHITECTURE OVERVIEW:
┌─────────────┐    JSON Commands    ┌─────────────────┐
│    ESP32    │ ──────────────────> │ Wokwi Simulator │
│ (Test Code) │ <────────────────── │   (Automation)  │
└─────────────┘    JSON Responses   └─────────────────┘

## CORE COMMUNICATION PRINCIPLE:
1. ESP32 sends JSON commands to Wokwi simulator via Tunnel
2. Wokwi paused execution, processes commands (button presses, sensor readings, GPIO state checks)
3. Wokwi sends JSON responses back to ESP32 and resumes execution
4. ESP32 continues test execution based on responses

## STRICT TUNNEL USAGE POLICY:
- All communication must go through the wokwi_send() and wokwi_receive() functions (or something similar)
- this will be implemented in the Wokwi simulator
- All other functions must use these two core functions
- All communication uses structured JSON format since it is easy to migrate this to other platforms (e.g. Python)

## JSON COMMAND EXAMPLES:
Button Press:    {"action":"set-control","part-id":"btn1","control":"pressed","value":1}
LED State Check: {"action":"get-control","part-id":"led1","control":"state"}
GPIO Output Value: {"action":"get-gpio-output-value","pin":4}
GPIO Pin Mode:   {"action":"get-gpio-mode","pin":4}
Delay:           {"action":"delay","duration":500} // probably not needed
MPU6050 Set Accel: {"action":"mpu6050-set-accel","part-id":"imu1","x":1.5,"y":0.0,"z":-9.81}
MPU6050 ... base on the https://docs.wokwi.com/parts/wokwi-mpu6050

In the future something like this, but let's start with the basics
SPI Init:        {"action":"spi-init","bus-id":"spi1","mode":0,"frequency":1000000}
SPI Send Test:   {"action":"spi-expect-data","bus-id":"spi1","data":"48656C6C6F","cs-pin":5}
SPI Failure:     {"action":"spi-simulate-failure","bus-id":"spi1","failure":"clock-stuck"}

## TYPICAL TEST WORKFLOW:
1. Configure GPIO pins (pinMode, attachInterrupt)
2. Use wokwi functions to simulate hardware interactions
3. Use Unity assertions to verify expected behavior
4. Library handles all simulator communication automatically

## SUPPORTED COMPONENTS:
- Push Buttons (press/release simulation)
- LEDs (state verification)
- Potentiometers (position control)
- MPU6050 Sensor (accelerometer, gyroscope, temperature)
- Generic Sensors (value injection)
- direct ESP32 GPIO pin output value and state (INPUT/OUTPUT)
- GPIO Interrupts (trigger detection)

In the future:
- Display verification
- Touch sensors

EXAMPLE USAGE:
```cpp
#include <wokwi.h>
#include <unity.h>
#include <wokwi_mpu6050.h>

// Test LED output
digitalWrite(LED_PIN, HIGH);
TEST_ASSERT_EQUAL(1, (int)wokwi_get_led_state("led1")); // Verify LED is on

// Test GPIO pin output value and mode
pinMode(LED_PIN, OUTPUT);
TEST_ASSERT_EQUAL(OUTPUT, wokwi_get_gpio_mode(LED_PIN)); // Verify pin mode
digitalWrite(LED_PIN, HIGH);
TEST_ASSERT_EQUAL(1, wokwi_get_gpio_output_value(LED_PIN)); // Verify output value

// Test MPU6050 sensor
wokwi_mpu6050_set_acceleration("imu1", 1.5, 0.0, 1.0); // Set acceleration
wokwi_mpu6050_set_temperature("imu1", 25.5);           // Set temperature
// Read sensor via I2C and verify values match
TEST_ASSERT_EQUAL_FLOAT(1.5, wokwi_mpu6050_get_acceleration_x("imu1"));
TEST_ASSERT_EQUAL_FLOAT(25.5, wokwi_mpu6050_get_temperature("imu1"));
```
