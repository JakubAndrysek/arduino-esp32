/*
 * WOKWI AUTOMATION LIBRARY - ESP32 GPIO TESTING
 *
 * This library provides a command-response pattern for automated testing of ESP32
 * functionality within the Wokwi simulator environment.
 *
 * ARCHITECTURE OVERVIEW:
 * ┌─────────────┐    JSON Commands    ┌─────────────────┐
 * │    ESP32    │ ──────────────────> │ Wokwi Simulator │
 * │ (Test Code) │ <────────────────── │   (Automation)  │
 * └─────────────┘    JSON Responses   └─────────────────┘
 *
 * CORE COMMUNICATION PRINCIPLE:
 * 1. ESP32 sends JSON commands to Wokwi simulator via Tunnel
 * 2. Wokwi paused execution, processes commands (button presses, sensor readings, GPIO state checks)
 * 3. Wokwi sends JSON responses back to ESP32 and resumes execution
 * 4. ESP32 continues test execution based on responses
 *
 * More details in README.md
 */

#include <unity.h>
#include <Arduino.h>
#include <stdarg.h>

#define WOKWI_TIMEOUT 5000  // Default timeout for simulator responses (ms)

// =============================================================================
// CORE COMMUNICATION FUNCTIONS (Only these access Tunnel directly)
// =============================================================================

/**
 * Send formatted command to Wokwi simulator
 * This is the ONLY function that should write to Tunnel
 */
int wokwi_send(const char *format, ...) {
  va_list args;
  va_start(args, format);
  int result = vprintf(format, args);
  va_end(args);
  Serial.flush();  // Ensure all data is sent before proceeding
  return result;
}

/**
 * Receive response from Wokwi simulator
 * This is the ONLY function that should read from Tunnel
 */
int wokwi_receive(char *buffer, size_t size) {
  unsigned long startTime = millis();
  size_t bytesRead = 0;

  while (millis() - startTime < WOKWI_TIMEOUT && bytesRead < size - 1) {
    if (Serial.available()) {
      buffer[bytesRead++] = Serial.read();
      if (buffer[bytesRead - 1] == '\n') {
        break;  // End of line
      }
    }
    delay(10);  // Avoid busy waiting
  }

  buffer[bytesRead] = '\0';  // Null-terminate the string
  return bytesRead;
}

// =============================================================================
// COMMAND-RESPONSE HELPER FUNCTIONS
// =============================================================================

/**
 * Wait for specific JSON response from simulator
 */
bool wokwi_wait_for_json_response(const char* expectedAction, const char* expectedStatus = "success") {
  char buffer[512];
  if (wokwi_receive(buffer, sizeof(buffer)) > 0) {
    if (strstr(buffer, expectedAction) != NULL && strstr(buffer, expectedStatus) != NULL) {
      return true;
    }
  }
  TEST_FAIL_MESSAGE("Timeout waiting for JSON response");
  return false;
}

/**
 * Parse numeric value from JSON response buffer
 */
double wokwi_parse_json_value_from_buffer(const char* jsonBuffer) {
  const char* valueStart = strstr(jsonBuffer, "\"value\":");
  if (valueStart != NULL) {
    valueStart += 8; // Skip "value":
    while (*valueStart == ' ' || *valueStart == '\t') valueStart++; // Skip whitespace
    return atof(valueStart);
  }
  return 0.0;
}

/**
 * Helper function to receive and parse JSON response with value
 * @param expectedAction - Expected action in response
 * @param expectedStatus - Expected status (default: "success")
 * @param value - Pointer to store parsed value (optional)
 * @return true if response received successfully, false on timeout/error
 */
bool wokwi_receive_json_response_with_value(const char* expectedAction, double* value = nullptr, const char* expectedStatus = "success") {
  char buffer[512];
  if (wokwi_receive(buffer, sizeof(buffer)) > 0) {
    if (strstr(buffer, expectedAction) != NULL && strstr(buffer, expectedStatus) != NULL) {
      if (value != nullptr) {
        *value = wokwi_parse_json_value_from_buffer(buffer);
      }
      return true;
    }
  }
  return false;
}

/**
 * Helper function to receive and parse JSON response with boolean value
 * @param expectedAction - Expected action in response
 * @param boolValue - Pointer to store parsed boolean value (optional)
 * @param expectedStatus - Expected status (default: "success")
 * @return true if response received successfully, false on timeout/error
 */
bool wokwi_receive_json_response_with_bool(const char* expectedAction, bool* boolValue = nullptr, const char* expectedStatus = "success") {
  char buffer[512];
  if (wokwi_receive(buffer, sizeof(buffer)) > 0) {
    if (strstr(buffer, expectedAction) != NULL && strstr(buffer, expectedStatus) != NULL) {
      if (boolValue != nullptr) {
        // Check for boolean values in JSON
        if (strstr(buffer, "\"exists\":true") != NULL || strstr(buffer, "\"result\":true") != NULL) {
          *boolValue = true;
        } else if (strstr(buffer, "\"exists\":false") != NULL || strstr(buffer, "\"result\":false") != NULL) {
          *boolValue = false;
        }
      }
      return true;
    }
  }
  return false;
}

// =============================================================================
// TIMING AND DELAY FUNCTIONS
// =============================================================================

/**
 * Send delay command to simulator and wait for completion
 * Note: This function blocks the ESP32 completely (including interrupts)
 * Use regular delay() for interrupt testing scenarios
 */
void wokwi_delay(int milliseconds) {
  wokwi_send("{\"action\":\"delay\",\"duration\":%d}\n", milliseconds);
  delay(milliseconds);
  wokwi_wait_for_json_response("delay");
}

// =============================================================================
// DIRECT GPIO CONTROL FUNCTIONS (read)
// =============================================================================

/**
 * Get GPIO pin output value (0 or 1)
 * @param pin - GPIO pin number
 * @return GPIO pin output value (0 = LOW, 1 = HIGH)
 */
int wokwi_get_gpio_output_value(int pin) {
  wokwi_send("{\"action\":\"get-gpio-output-value\",\"pin\":%d}\n", pin);

  double value;
  if (wokwi_receive_json_response_with_value("get-gpio-output-value", &value)) {
    return (int)value;
  }

  TEST_FAIL_MESSAGE("Timeout waiting for get-gpio-output-value response");
  return 0;
}

/**
 * Get GPIO pin mode/state using Arduino defines
 * @param pin - GPIO pin number
 * @return GPIO pin mode (INPUT=0x01, OUTPUT=0x03, INPUT_PULLUP=0x05, etc.)
 * Returns the actual Arduino mode value set by pinMode()
 */
int wokwi_get_gpio_mode(int pin) {
  wokwi_send("{\"action\":\"get-gpio-mode\",\"pin\":%d}\n", pin);

  double value;
  if (wokwi_receive_json_response_with_value("get-gpio-mode", &value)) {
    return (int)value;
  }

  TEST_FAIL_MESSAGE("Timeout waiting for get-gpio-mode response");
  return INPUT; // Default to INPUT if timeout
}

/**
 * Get GPIO pin input value (0 or 1)
 * @param pin - GPIO pin number
 * @return GPIO pin input value (0 = LOW, 1 = HIGH)
 */
int wokwi_get_gpio_input_value(int pin) {
  wokwi_send("{\"action\":\"get-gpio-input-value\",\"pin\":%d}\n", pin);

  double value;
  if (wokwi_receive_json_response_with_value("get-gpio-input-value", &value)) {
    return (int)value;
  }

  TEST_FAIL_MESSAGE("Timeout waiting for get-gpio-input-value response");
  return 0;
}

// =============================================================================
// COMPONENT CONTROL FUNCTIONS
// =============================================================================

/**
 * Set component control value (integer)
 */
void wokwi_set_control_int(const char* partId, const char* control, int value) {
  wokwi_send("{\"action\":\"set-control\",\"part-id\":\"%s\",\"control\":\"%s\",\"value\":%d}\n",
             partId, control, value);
  wokwi_wait_for_json_response("set-control");
}

/**
 * Set component control value (float)
 */
void wokwi_set_control_float(const char* partId, const char* control, float value) {
  wokwi_send("{\"action\":\"set-control\",\"part-id\":\"%s\",\"control\":\"%s\",\"value\":%.2f}\n",
             partId, control, value);
  wokwi_wait_for_json_response("set-control");
}

/**
 * Get component control value
 */
double wokwi_get_control(const char *partId, const char *control) {
  wokwi_send("{\"action\":\"get-control\",\"part-id\":\"%s\",\"control\":\"%s\"}\n", partId, control);

  double value;
  if (wokwi_receive_json_response_with_value("get-control", &value)) {
    return value;
  }

  TEST_FAIL_MESSAGE("Timeout waiting for get-control response");
  return 0.0;
}

// =============================================================================
// BUTTON SIMULATION FUNCTIONS
// =============================================================================

/**
 * Press button (set pressed state to 1)
 */
void wokwi_press_button(const char* partId) {
  wokwi_set_control_int(partId, "pressed", 1);
}

/**
 * Release button (set pressed state to 0)
 */
void wokwi_release_button(const char* partId) {
  wokwi_set_control_int(partId, "pressed", 0);
}


/**
 * Simulate button press for interrupt testing
 * This function presses the button, waits for a specified time, then releases it.
 * It allows the ESP32 to process interrupts while the button is pressed.
 * @param partId - Component ID of the button (e.g., "btn1")
 * @param holdTimeMs - Duration to hold the button pressed (default: 100ms)
 */
void wokwi_simulate_button_press(const char* partId, int holdTimeMs = 100) {
  wokwi_press_button(partId);
  delay(holdTimeMs);                  // ESP32 waits but can process interrupts
  wokwi_release_button(partId);
}

// =============================================================================
// LED VERIFICATION FUNCTIONS
// =============================================================================

/**
 * Get LED state (0 = OFF, 1 = ON)
 */
double wokwi_get_led_state(const char *partId) {
  return wokwi_get_control(partId, "state");
}


// =============================================================================
// SENSOR AND COMPONENT CONTROL FUNCTIONS
// =============================================================================

/**
 * Set potentiometer position (0-1023)
 */
void wokwi_set_potentiometer(const char *partId, int position) {
  wokwi_set_control_int(partId, "position", position);
}

/**
 * Set sensor value
 */
void wokwi_set_sensor_value(const char *partId, const char *sensor, float value) {
  wokwi_set_control_float(partId, sensor, value);
}

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
