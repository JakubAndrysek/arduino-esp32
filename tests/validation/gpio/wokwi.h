/*
 * WOKWI AUTOMATION LIBRARY - ESP32 GPIO TESTING
 *
 * This library provides a command-response pattern for automated testing of ESP32 GPIO
 * functionality within the Wokwi simulator environment. It enables comprehensive testing
 * of digital I/O, interrupts, and hardware components through programmatic control.
 *
 * ARCHITECTURE OVERVIEW:
 * ┌─────────────┐    JSON Commands    ┌─────────────────┐
 * │    ESP32    │ ──────────────────> │ Wokwi Simulator │
 * │ (Test Code) │ <────────────────── │   (Automation)  │
 * └─────────────┘    JSON Responses   └─────────────────┘
 *
 * CORE COMMUNICATION PRINCIPLE:
 * 1. ESP32 sends JSON commands to Wokwi simulator via Serial
 * 2. Wokwi processes commands (button presses, sensor readings, GPIO state checks)
 * 3. Wokwi sends JSON responses back to ESP32
 * 4. ESP32 continues test execution based on responses
 *
 * STRICT SERIAL USAGE POLICY:
 * - Only wokwi_send() and wokwi_receive() access Serial directly
 * - All other functions must use these two core functions
 * - No debug output to ESP32 Serial (except test failures via TEST_FAIL_MESSAGE)
 * - All communication uses structured JSON format
 *
 * JSON COMMAND EXAMPLES:
 * Button Press:    {"action":"set-control","part-id":"btn1","control":"pressed","value":1}
 * LED State Check: {"action":"get-control","part-id":"led1","control":"state"}
 * Delay:           {"action":"delay","duration":500}
 * Interrupt Wait:  {"action":"wait-for-interrupt","interrupt":"gpio","timeout":5000}
 *
 * TYPICAL TEST WORKFLOW:
 * 1. Configure GPIO pins (pinMode, attachInterrupt)
 * 2. Use wokwi functions to simulate hardware interactions
 * 3. Use Unity assertions to verify expected behavior
 * 4. Library handles all simulator communication automatically
 *
 * SUPPORTED COMPONENTS:
 * - Push Buttons (press/release simulation)
 * - LEDs (state verification)
 * - Potentiometers (position control)
 * - Sensors (value injection)
 * - GPIO Interrupts (trigger detection)
 *
 * EXAMPLE USAGE:
 *   // Test button interrupt
 *   attachInterrupt(digitalPinToInterrupt(BTN_PIN), buttonISR, FALLING);
 *   wokwi_press_button("btn1");                    // Simulate button press
 *   TEST_ASSERT_TRUE(wokwi_wait_for_interrupt());  // Wait for interrupt trigger
 *   TEST_ASSERT_EQUAL(1, interruptCounter);       // Verify interrupt occurred
 *
 *   // Test LED output
 *   digitalWrite(LED_PIN, HIGH);
 *   TEST_ASSERT_EQUAL(1, (int)wokwi_get_led_state("led1")); // Verify LED is on
 */

#include <unity.h>
#include <Arduino.h>
#include <stdarg.h>

#define WOKWI_TIMEOUT 5000  // Default timeout for simulator responses (ms)

// =============================================================================
// CORE COMMUNICATION FUNCTIONS (Only these access Serial directly)
// =============================================================================

/**
 * Send formatted command to Wokwi simulator
 * This is the ONLY function that should write to Serial
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
 * This is the ONLY function that should read from Serial
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

// =============================================================================
// TIMING AND DELAY FUNCTIONS
// =============================================================================

/**
 * Send delay command to simulator and wait for completion
 */
void wokwi_delay(int milliseconds) {
  wokwi_send("{\"action\":\"delay\",\"duration\":%d}\n", milliseconds);
  delay(milliseconds);  // Also delay locally
  wokwi_wait_for_json_response("delay");
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

  char buffer[512];
  if (wokwi_receive(buffer, sizeof(buffer)) > 0) {
    if (strstr(buffer, "get-control") != NULL && strstr(buffer, "success") != NULL) {
      return wokwi_parse_json_value_from_buffer(buffer);
    }
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
 * Simulate complete button press cycle (press, hold, release)
 */
void wokwi_simulate_button_press(const char* partId, int holdTimeMs = 100) {
  wokwi_press_button(partId);
  wokwi_delay(holdTimeMs);
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
// INTERRUPT TESTING FUNCTIONS
// =============================================================================

/**
 * Wait for interrupt detection by simulator
 * Returns true when simulator detects interrupt trigger, false on timeout
 */
bool wokwi_wait_for_interrupt(int timeoutMs = 5000, const char* interruptName = "interrupt") {
  wokwi_send("{\"action\":\"wait-for-interrupt\",\"interrupt\":\"%s\",\"timeout\":%d}\n",
             interruptName, timeoutMs);

  unsigned long startTime = millis();
  char responseBuffer[256];

  while (millis() - startTime < timeoutMs) {
    if (wokwi_receive(responseBuffer, sizeof(responseBuffer)) > 0) {
      if (strstr(responseBuffer, "\"action\":\"interrupt-detected\"") != NULL) {
        delay(50);  // Allow time for interrupt handler to execute
        return true;
      }
    }
    delay(10);
  }

  TEST_FAIL_MESSAGE("Timeout waiting for interrupt");
  return false;
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
