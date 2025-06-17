// ---
// title: Wokwi Automation Scenarios
// sidebar_label: Automation Scenarios
// ---

// Automation scenarios allow you to automate the simulation, push buttons, change the state of the sensors, and check the serial output. You can use automation scenarios to test your firmware in a realistic environment, and verify that it behaves as expected.

// Each automation scenario is a YAML file that describes a sequence of actions that the simulator should take. You can use the `--scenario` CLI option to load an automation scenario file.

// The basic structure of an automation scenario file is as follows:

// ```yaml
// name: 'Your scenario name'
// version: 1
// author: 'Your name'

// steps:
//   # List of steps:
//   - set-control:
//       part-id: btn1
//       control: pressed
//       value: 1
//   - delay: 500ms
//   - wait-serial: 'Button 1 pressed'
// ```

// :::warning
// Automation scenarios are currently in alpha. The API is not fully documented yet, and may change in the future. You can use the [example projects](github-actions#examples) as a reference.
// :::

// ## Supported Parts with Automation Controls

// Several Wokwi parts support automation controls that can be controlled using automation scenarios. These controls allow you to programmatically change the state of sensors, press buttons, and modify component values during simulation.

// ### Parts with Automation Controls

// - **[Push Button](../parts/wokwi-pushbutton#automation-controls)** - Control button presses and releases
// - **[Potentiometer](../parts/wokwi-potentiometer#automation-controls)** - Adjust the potentiometer position
// - **[MPU6050 Sensor](../parts/wokwi-mpu6050#automation-controls)** - Set acceleration, rotation, and temperature values

// Each part's documentation page contains detailed information about the specific automation controls available, including control names, types, and example usage.

// steps:
//   ##########################################################
//   #### GPIO interrupt test
//   ##########################################################

//   - wait-serial: "GPIO interrupt test START"
//   #### GPIO interrupt - attach/detach test
//   - wait-serial: "GPIO interrupt - attach/detach test START"
//   - wait-serial: "Interrupt attached - FALLING edge"
//   - wait-serial: "Press button to trigger interrupt"

//   # Simulate button press (falling edge)
//   - set-control:
//       part-id: btn1
//       control: pressed
//       value: 1

//   - wait-serial: "First interrupt triggered successfully"
//   - wait-serial: "Press button again to trigger second interrupt"

// This is now proposal for a Wokwi automation scenario header file in C++
// The aim is to provide built-in support for Wokwi automation scenarios directly from source code
// Defines and functions will execute the action and wait for the response from the Wokwi simulator
// This will allow users to write tests that can be run in the Wokwi simulator without needing to write YAML files
// The benefit is that users won't need manually synchronize running the tests with the scenario steps
// All blocks are blocking, meaning they will wait until the Wokwi action (e.g., button press) is completed
//
// ** JSON Communication Format **
// All communication messages now use JSON format for better structure and parsing:
// - Set commands: {"action": "set-control", "part-id": "btn1", "control": "pressed", "value": 1}
// - Get commands: {"action": "get-control", "part-id": "pot1", "control": "position"}
// - Expected responses: {"action": "set-control", "status": "success", ...}
// - Get responses: {"action": "get-control", "status": "success", "value": 123.45}
// - Wait commands: {"action": "wait-serial", "type": "message", "content": "Button pressed"}
// - Delay commands: {"action": "delay", "duration": 500}
//
// Example usage with JSON:
// wokwi_press_button("btn1");           // Sends: {"action":"set-control","part-id":"btn1","control":"pressed","value":1}
// wokwi_delay(500);                     // Sends: {"action":"delay","duration":500}
// wokwi_set_potentiometer("pot1", 512); // Sends: {"action":"set-control","part-id":"pot1","control":"position","value":512}
// double value = wokwi_get_control("pot1", "position"); // Sends: {"action":"get-control","part-id":"pot1","control":"position"}
//                                       // Receives: {"action":"get-control","status":"success","value":512.0}
// double temp = wokwi_get_sensor_value("mpu1", "temperature"); // Get sensor temperature value

#include <unity.h>
#include <Arduino.h>
#include <stdarg.h>

#define WOKWI_TIMEOUT 5000  // Timeout for waiting for serial messages

int wokwi_send(const char *format, ...) {
  va_list args;
  va_start(args, format);
  int result = vprintf(format, args);
  va_end(args);
  Serial.flush();  // Ensure all data is sent before proceeding
  return result;
}

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


// Legacy function - now uses command-response pattern
void wokwi_wait_for_serial(const char *message) {
  // This function is deprecated - commands now use request-response pattern
  TEST_FAIL_MESSAGE("wokwi_wait_for_serial is deprecated - use command-response pattern");
}

void wokwi_wait_for_json_serial(const char *messageType, const char *content) {
  // Send JSON wait command and wait for response
  wokwi_send("{\"action\":\"wait-serial\",\"type\":\"%s\",\"content\":\"%s\"}\n", messageType, content);

  if (!wokwi_wait_for_json_response("wait-serial", "success")) {
    TEST_FAIL_MESSAGE("Failed to execute wait-serial command");
  }
}

// Legacy function - now uses command-response pattern
void wokwi_wait_response(const char *response) {
  // This function is deprecated - use wokwi_wait_for_json_response instead
  TEST_FAIL_MESSAGE("wokwi_wait_response is deprecated - use wokwi_wait_for_json_response");
}

// This function is now defined above and uses only wokwi_receive

void wokwi_set_control(const char *partId, const char *control, int value) {
  // Use the new structured approach
  wokwi_send_set_control_int(partId, control, value);
}

double wokwi_get_control(const char *partId, const char *control) {
  // Send JSON command to get control value
  wokwi_send("{\"action\":\"get-control\",\"part-id\":\"%s\",\"control\":\"%s\"}\n", partId, control);

  // Wait for and parse the JSON response using new pattern
  char buffer[512];
  if (wokwi_receive(buffer, sizeof(buffer)) > 0) {
    if (strstr(buffer, "get-control") != NULL && strstr(buffer, "success") != NULL) {
      return wokwi_parse_json_value_from_buffer(buffer);
    }
  }
  TEST_FAIL_MESSAGE("Timeout waiting for get-control response");
  return 0.0;
}

void wokwi_delay(int milliseconds) {
  // Send JSON command
  wokwi_send("{\"action\":\"delay\",\"duration\":%d}\n", milliseconds);
  delay(milliseconds);  // Simulate delay
  wokwi_wait_json_response("delay");
}

// Core set-control function with proper parameter handling
void wokwi_send_set_control(const char* partId, const char* control, const char* valueStr) {
  wokwi_send("{\"action\":\"set-control\",\"part-id\":\"%s\",\"control\":\"%s\",\"value\":%s}\n", partId, control, valueStr);
  wokwi_wait_json_response("set-control");
}

// Overloaded functions for different value types
void wokwi_send_set_control_int(const char* partId, const char* control, int value) {
  char valueStr[32];
  sprintf(valueStr, "%d", value);
  wokwi_send_set_control(partId, control, valueStr);
}

void wokwi_send_set_control_float(const char* partId, const char* control, float value) {
  char valueStr[32];
  sprintf(valueStr, "%.2f", value);
  wokwi_send_set_control(partId, control, valueStr);
}

void wokwi_send_set_control_bool(const char* partId, const char* control, bool value) {
  wokwi_send_set_control_int(partId, control, value ? 1 : 0);
}

void wokwi_send_set_control_string(const char* partId, const char* control, const char* value) {
  char valueStr[256];
  sprintf(valueStr, "\"%s\"", value);
  wokwi_send_set_control(partId, control, valueStr);
}

// Additional JSON-based helper functions

void wokwi_send_json_command(const char *action, const char *jsonParams = "") {
  if (strlen(jsonParams) > 0) {
    wokwi_send("{\"action\":\"%s\",%s}\n", action, jsonParams);
  } else {
    wokwi_send("{\"action\":\"%s\"}\n", action);
  }
  wokwi_wait_json_response(action);
}

void wokwi_set_potentiometer(const char *partId, int position) {
  // Send JSON command for potentiometer control (0-1023) using structured approach
  wokwi_send_set_control_int(partId, "position", position);
}

void wokwi_set_sensor_value(const char *partId, const char *sensor, float value) {
  // Send JSON command for sensor value setting using structured approach
  wokwi_send_set_control_float(partId, sensor, value);
}

// LED control functions
void wokwi_set_led(const char *partId, int state) {
  // Send JSON command for LED control (0 = OFF, 1 = ON) using structured approach
  wokwi_send_set_control_int(partId, "state", state);
}

double wokwi_get_led_state(const char *partId) {
  return wokwi_get_control(partId, "state");
}


// Wait for interrupt trigger with timeout and detailed logging
bool wokwi_wait_for_interrupt(int timeoutMs = 5000, const char* interruptName = "interrupt") {
  // Send JSON command to start monitoring for interrupt
  wokwi_send("{\"action\":\"wait-for-interrupt\",\"interrupt\":\"%s\",\"timeout\":%d}\n", interruptName, timeoutMs);

  unsigned long startTime = millis();
  char responseBuffer[256];

  while (millis() - startTime < timeoutMs) {
    // Continuously read for simulator responses
    if (wokwi_receive(responseBuffer, sizeof(responseBuffer)) > 0) {
      // Check if simulator detected the interrupt event
      if (strstr(responseBuffer, "\"action\":\"interrupt-detected\"") != NULL) {
        // Wait a bit for the actual interrupt handler to execute
        delay(50);
        return true;
      }
    }
    delay(10);
  }

  // Timeout occurred
  TEST_FAIL_MESSAGE("Timeout waiting for interrupt");
  return false;
}

// Core logging function with proper parameter handling
void wokwi_send_log_message(const char* logLevel, const char* message) {
  wokwi_send("{\"action\":\"log-%s\",\"message\":\"%s\"}\n", logLevel, message);
}

// Decomposed logging functions using structured approach
void wokwi_log_error(const char* errorMessage) {
  wokwi_send_log_message("error", errorMessage);
}

void wokwi_log_warning(const char* warningMessage) {
  wokwi_send_log_message("warning", warningMessage);
}

void wokwi_log_info(const char* infoMessage) {
  wokwi_send_log_message("info", infoMessage);
}

void wokwi_log_debug(const char* debugMessage) {
  wokwi_send_log_message("debug", debugMessage);
}

// Core component action function
void wokwi_send_component_action(const char* action, const char* partId, const char* extraParams = "") {
  if (strlen(extraParams) > 0) {
    wokwi_send("{\"action\":\"%s\",\"part-id\":\"%s\",%s}\n", action, partId, extraParams);
  } else {
    wokwi_send("{\"action\":\"%s\",\"part-id\":\"%s\"}\n", action, partId);
  }
  wokwi_wait_json_response(action);
}

// Decomposed component functions using structured approach
void wokwi_reset_component(const char* partId) {
  wokwi_send_component_action("reset-component", partId);
}

// Core test action function
void wokwi_send_test_action(const char* action, const char* message) {
  wokwi_send("{\"action\":\"%s\",\"message\":\"%s\"}\n", action, message);
}

// Decomposed test functions using structured approach
void wokwi_test_fail(const char* message) {
  wokwi_send_test_action("test-fail", message);
  TEST_FAIL_MESSAGE(message);  // Only print failures
  UNITY_END();  // End Unity test session
}

void wokwi_test_pass(const char* message) {
  wokwi_send_test_action("test-pass", message);
  // Don't print success messages to ESP serial
}

void wokwi_test_skip(const char* message) {
  wokwi_send_test_action("test-skip", message);
  // Don't print skip messages to ESP serial
}

// Additional component action functions using structured approach
void wokwi_enable_component(const char* partId) {
  wokwi_send_component_action("enable-component", partId);
}

void wokwi_disable_component(const char* partId) {
  wokwi_send_component_action("disable-component", partId);
}

void wokwi_configure_component(const char* partId, const char* config) {
  char extraParams[256];
  sprintf(extraParams, "\"config\":\"%s\"", config);
  wokwi_send_component_action("configure-component", partId, extraParams);
  // Don't print configuration details to ESP serial
}

// JSON response parsing helper function
double wokwi_parse_json_value(const String &jsonResponse) {
  int valueStart = jsonResponse.indexOf("\"value\":");
  if (valueStart >= 0) {
    valueStart += 8;  // Move past "value":
    // Skip whitespace
    while (valueStart < jsonResponse.length() && (jsonResponse.charAt(valueStart) == ' ' || jsonResponse.charAt(valueStart) == '\t')) {
      valueStart++;
    }

    int valueEnd = jsonResponse.indexOf(',', valueStart);
    if (valueEnd == -1) {
      valueEnd = jsonResponse.indexOf('}', valueStart);
    }
    if (valueEnd == -1) {
      valueEnd = jsonResponse.length();
    }

    String valueStr = jsonResponse.substring(valueStart, valueEnd);
    valueStr.trim();
    return valueStr.toDouble();
  }
  return 0.0;
}

// Button control functions using structured approach
void wokwi_press_button(const char* partId) {
  // Press button (set to 1)
  wokwi_send_set_control_int(partId, "pressed", 1);
}

void wokwi_release_button(const char* partId) {
  // Release button (set to 0)
  wokwi_send_set_control_int(partId, "pressed", 0);
}

// Button simulation with automatic press and release
void wokwi_simulate_button_press(const char* partId, int holdTimeMs = 100) {
  wokwi_press_button(partId);
  wokwi_delay(holdTimeMs);
  wokwi_release_button(partId);
}

// Component state verification
// probably should be needed, we will test the response using pytest
bool wokwi_verify_control_state(const char* partId, const char* control, double expectedValue, double tolerance = 0.01) {
  double actualValue = wokwi_get_control(partId, control);
  bool matches = abs(actualValue - expectedValue) <= tolerance;

  if (matches) {
    wokwi_send("{\"action\":\"verify-success\",\"part-id\":\"%s\",\"control\":\"%s\",\"expected\":%.2f,\"actual\":%.2f}\n",
           partId, control, expectedValue, actualValue);
  } else {
    wokwi_send("{\"action\":\"verify-failed\",\"part-id\":\"%s\",\"control\":\"%s\",\"expected\":%.2f,\"actual\":%.2f}\n",
           partId, control, expectedValue, actualValue);
  }

  return matches;
}

// Core command-response functions - only these use Serial directly
bool wokwi_send_command_and_wait_response(const char* command, char* response, size_t responseSize) {
  wokwi_send("%s", command);
  int bytesReceived = wokwi_receive(response, responseSize);
  return bytesReceived > 0;
}

bool wokwi_wait_for_json_response(const char* expectedAction, const char* expectedStatus) {
  char buffer[512];
  if (wokwi_receive(buffer, sizeof(buffer)) > 0) {
    // Simple JSON parsing - check if response contains expected action and status
    if (strstr(buffer, expectedAction) != NULL && strstr(buffer, expectedStatus) != NULL) {
      return true;
    }
  }
  TEST_FAIL_MESSAGE("Timeout waiting for JSON response");
  return false;
}

// Legacy function name compatibility
void wokwi_wait_json_response(const char* action, const char* status = "success") {
  if (!wokwi_wait_for_json_response(action, status)) {
    TEST_FAIL_MESSAGE("Failed to receive expected JSON response");
  }
}

double wokwi_parse_json_value_from_buffer(const char* jsonBuffer) {
  // Simple JSON value extraction
  const char* valueStart = strstr(jsonBuffer, "\"value\":");
  if (valueStart != NULL) {
    valueStart += 8; // Skip "value":
    // Skip whitespace
    while (*valueStart == ' ' || *valueStart == '\t') valueStart++;
    return atof(valueStart);
  }
  return 0.0;
}

/*
 * WOKWI AUTOMATION LIBRARY - COMMAND-RESPONSE ARCHITECTURE
 *
 * This library implements a command-response pattern for Wokwi automation testing.
 *
 * CORE PRINCIPLE:
 * - ESP sends commands to Wokwi simulator
 * - Wokwi processes the command (e.g., reads GPIO state, simulates button press)
 * - Wokwi sends back response with requested data
 * - ESP continues based on response
 *
 * SERIAL USAGE POLICY:
 * - Only wokwi_send() and wokwi_receive() use Serial directly
 * - All other functions use these two core functions
 * - No debug output to ESP serial (except test failures)
 * - All communication is JSON-based
 *
 * EXAMPLE USAGE:
 * 1. ESP: digitalWrite(5, HIGH)
 * 2. ESP: wokwi_get_control("led1", "state")
 *    -> Sends: {"action":"get-control","part-id":"led1","control":"state"}
 *    <- Receives: {"action":"get-control","status":"success","value":1.0}
 * 3. ESP: Continues with returned value
 *
 * This allows automated verification of ESP behavior without manual interaction.
 */
