#include <Arduino.h>
#include <unity.h>

#define BTN 0
#define LED 4

void setUp(void) {}

void tearDown(void) {}

void test_read_basic(void) {
  Serial.println("GPIO read - basic START");

  pinMode(BTN, INPUT_PULLUP);
  assert(digitalRead(BTN) == 1);
  TEST_ASSERT_EQUAL(HIGH, digitalRead(BTN));
  Serial.println("BTN read as HIGH after pinMode INPUT_PULLUP");

  delay(1000);
  TEST_ASSERT_EQUAL(LOW, digitalRead(BTN));
  Serial.println("BTN read as LOW");

  delay(1000);
  TEST_ASSERT_EQUAL(HIGH, digitalRead(BTN));
  Serial.println("BTN read as HIGH");
}

void test_write_basic(void) {
  Serial.println("GPIO write - basic test");
  pinMode(LED, OUTPUT);
  delay(1000);
  Serial.println("GPIO LED set to OUTPUT");
  delay(2000);

  digitalWrite(LED, HIGH);
  delay(1000);
  Serial.println("LED set to HIGH");

  delay(3000);
  digitalWrite(LED, LOW);
  Serial.println("LED set to LOW");
}


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  UNITY_BEGIN();
  RUN_TEST(test_read_basic);
  RUN_TEST(test_write_basic);
  UNITY_END();

  Serial.println("GPIO test END");
}

void loop() {}
