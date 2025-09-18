#include <Arduino.h>
#include <unity.h>

#define BTN 0
#define LED 4

#if CONFIG_IDF_TARGET_ESP32P4
#define POT_PIN 53
#else
#define POT_PIN 2
#endif



void setUp(void) {}

void tearDown(void) {}

void test_adc_potentiometer(void) {
  Serial.println("ADC potentiometer test START");

  // Configure ADC
  analogReadResolution(12);  // Set ADC resolution to 12 bits (0-4095)
  analogSetAttenuation(ADC_11db);  // Set attenuation for full 3.3V range

  Serial.println("Reading ADC values from potentiometer on GPIO2");

  // Take multiple readings to test different potentiometer positions
  for (int i = 0; i < 5; i++) {
    int adcValue = analogRead(POT_PIN);
    float voltage = (adcValue * 3.3) / 4095.0;

    Serial.print("ADC Reading ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(adcValue);
    Serial.print(" (");
    Serial.print(voltage, 2);
    Serial.println("V)");

    // Test that ADC reading is within valid range
    TEST_ASSERT_GREATER_OR_EQUAL(0, adcValue);
    TEST_ASSERT_LESS_OR_EQUAL(4095, adcValue);

    delay(1000);  // Wait 1 second between readings
  }

  Serial.println("ADC potentiometer test completed successfully");
}


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  UNITY_BEGIN();
  RUN_TEST(test_adc_potentiometer);
  UNITY_END();

  Serial.println("ADC test END");
}

void loop() {}
