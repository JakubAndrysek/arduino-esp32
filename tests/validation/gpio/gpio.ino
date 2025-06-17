#include <Arduino.h>
#include <unity.h>
#include <wokwi.h>


#define BTN 0
#define LED 4

volatile int interruptCounter = 0;
volatile bool interruptFlag = false;
volatile unsigned long lastInterruptTime = 0;

// Variables for interrupt with argument test
volatile int argInterruptCounter = 0;
volatile bool argInterruptFlag = false;
volatile int receivedArg = 0;

void setUp(void) {
  interruptCounter = 0;
  interruptFlag = false;
  lastInterruptTime = 0;
  argInterruptCounter = 0;
  argInterruptFlag = false;
  receivedArg = 0;
}

void tearDown(void) {
  detachInterrupt(digitalPinToInterrupt(BTN));
}

void IRAM_ATTR buttonISR() {
  unsigned long currentTime = millis();
  // Simple debouncing - ignore interrupts within 50ms
  if (currentTime - lastInterruptTime > 50) {
    interruptCounter = interruptCounter + 1;
    interruptFlag = true;
    lastInterruptTime = currentTime;
  }
}

void IRAM_ATTR buttonISRWithArg(void *arg) {
  unsigned long currentTime = millis();
  // Simple debouncing - ignore interrupts within 50ms
  if (currentTime - lastInterruptTime > 50) {
    argInterruptCounter = argInterruptCounter + 1;
    argInterruptFlag = true;
    receivedArg = *(int*)arg;
    lastInterruptTime = currentTime;
  }
}

void test_interrupt_attach_detach(void) {
  wokwi_wait_for_serial("GPIO interrupt - attach/detach test START");

  pinMode(BTN, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  // Reset counters
  interruptCounter = 0;
  interruptFlag = false;

  // Attach interrupt on falling edge (button press)
  attachInterrupt(digitalPinToInterrupt(BTN), buttonISR, FALLING);

  // Simulate button press using Wokwi
  wokwi_simulate_button_press("btn1", 200);

  // Wait for interrupt to be triggered
  TEST_ASSERT_TRUE(wokwi_wait_for_interrupt(5000));
  TEST_ASSERT_EQUAL(1, interruptCounter);

  // Reset flag for next test
  interruptFlag = false;

  // Simulate second button press
  wokwi_simulate_button_press("btn1", 200);

  TEST_ASSERT_TRUE(wokwi_wait_for_interrupt(5000));
  TEST_ASSERT_EQUAL(2, interruptCounter);

  detachInterrupt(digitalPinToInterrupt(BTN));

  // Reset counters and test that interrupt no longer works
  interruptCounter = 0;
  interruptFlag = false;

  // Simulate button press (should not trigger interrupt)
  wokwi_simulate_button_press("btn1", 200);
  wokwi_delay(3000);

  TEST_ASSERT_FALSE(interruptFlag);
  TEST_ASSERT_EQUAL(0, interruptCounter);
}

void test_interrupt_rising_falling(void) {
  pinMode(BTN, INPUT_PULLUP);

  // Test FALLING edge
  interruptCounter = 0;
  interruptFlag = false;

  attachInterrupt(digitalPinToInterrupt(BTN), buttonISR, FALLING);

  // Simulate button press (falling edge)
  wokwi_press_button("btn1");

  TEST_ASSERT_TRUE(wokwi_wait_for_interrupt(5000));

  // Release button for next test
  wokwi_release_button("btn1");
  detachInterrupt(digitalPinToInterrupt(BTN));

  // Test RISING edge
  interruptCounter = 0;
  interruptFlag = false;

  attachInterrupt(digitalPinToInterrupt(BTN), buttonISR, RISING);

  // First press button (no interrupt expected)
  wokwi_press_button("btn1");
  wokwi_delay(500);

  // Then release button (rising edge - should trigger interrupt)
  wokwi_release_button("btn1");

  TEST_ASSERT_TRUE(wokwi_wait_for_interrupt(5000));

  detachInterrupt(digitalPinToInterrupt(BTN));
}

void test_interrupt_change(void) {
  pinMode(BTN, INPUT_PULLUP);

  interruptCounter = 0;
  interruptFlag = false;

  attachInterrupt(digitalPinToInterrupt(BTN), buttonISR, CHANGE);

  // Simulate button press (falling edge - should trigger interrupt)
  wokwi_press_button("btn1");

  // Wait for button press (falling edge)
  TEST_ASSERT_TRUE(wokwi_wait_for_interrupt(5000));
  TEST_ASSERT_GREATER_OR_EQUAL(1, interruptCounter);

  // Reset flag for release detection
  interruptFlag = false;

  // Simulate button release (rising edge - should trigger interrupt again)
  wokwi_release_button("btn1");

  // Wait for button release (rising edge)
  TEST_ASSERT_TRUE(wokwi_wait_for_interrupt(5000));
  TEST_ASSERT_GREATER_OR_EQUAL(2, interruptCounter);

  detachInterrupt(digitalPinToInterrupt(BTN));
}

void test_interrupt_with_arg(void) {
  pinMode(BTN, INPUT_PULLUP);

  // Test data to pass to interrupt
  int testArg = 42;

  // Reset counters
  argInterruptCounter = 0;
  argInterruptFlag = false;
  receivedArg = 0;

  // Attach interrupt with argument on falling edge (button press)
  attachInterruptArg(digitalPinToInterrupt(BTN), buttonISRWithArg, &testArg, FALLING);

  // Simulate button press
  wokwi_simulate_button_press("btn1", 200);

  // Wait for interrupt to be triggered
  TEST_ASSERT_TRUE(wokwi_wait_for_interrupt(5000));
  TEST_ASSERT_EQUAL(1, argInterruptCounter);
  TEST_ASSERT_EQUAL(42, receivedArg);

  // Test with different argument value
  argInterruptFlag = false;
  int newTestArg = 123;

  // Detach and reattach with new argument
  detachInterrupt(digitalPinToInterrupt(BTN));
  attachInterruptArg(digitalPinToInterrupt(BTN), buttonISRWithArg, &newTestArg, FALLING);

  // Simulate second button press
  wokwi_simulate_button_press("btn1", 200);

  TEST_ASSERT_TRUE(wokwi_wait_for_interrupt(5000));
  TEST_ASSERT_EQUAL(2, argInterruptCounter);
  TEST_ASSERT_EQUAL(123, receivedArg);

  detachInterrupt(digitalPinToInterrupt(BTN));
}


void test_read_basic(void) {
  pinMode(BTN, INPUT_PULLUP);
  TEST_ASSERT_EQUAL(HIGH, digitalRead(BTN));

  wokwi_press_button("btn1");

  TEST_ASSERT_EQUAL(LOW, digitalRead(BTN));

  wokwi_release_button("btn1");
  TEST_ASSERT_EQUAL(HIGH, digitalRead(BTN));
}

void test_write_basic(void) {
  pinMode(LED, OUTPUT);

  // Check initial LED state
  double initialState = wokwi_get_led_state("led1");
  TEST_ASSERT_EQUAL(LOW, (int)initialState); // Ensure LED is off initially

  digitalWrite(LED, HIGH);

  // Verify LED state through Wokwi
  double ledState = wokwi_get_led_state("led1");
  TEST_ASSERT_EQUAL(HIGH, (int)ledState);

  wokwi_delay(3000);
  digitalWrite(LED, LOW);

  // Verify LED is now off
  ledState = wokwi_get_led_state("led1");
  TEST_ASSERT_EQUAL(LOW, (int)ledState);
}


void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  UNITY_BEGIN();

  Serial.println("GPIO interrupt test START");
  RUN_TEST(test_interrupt_attach_detach);
  RUN_TEST(test_interrupt_rising_falling);
  RUN_TEST(test_interrupt_change);
  RUN_TEST(test_interrupt_with_arg);
  Serial.println("GPIO interrupt test END");

  Serial.println("GPIO basic test START");
  RUN_TEST(test_read_basic);
  RUN_TEST(test_write_basic);
  Serial.println("GPIO basic test END");

  UNITY_END();
  Serial.println("GPIO test END");
}

void loop() {}
