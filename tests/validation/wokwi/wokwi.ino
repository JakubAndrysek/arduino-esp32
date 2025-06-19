#include <Arduino.h>

#define BTN 0
#define LED 2

void test_button() {
  bool buttonState = digitalRead(BTN);
  if (buttonState == LOW) {  // Button pressed (INPUT_PULLUP means LOW when pressed)
    digitalWrite(LED, HIGH);
    Serial.println("LED ON");
  } else {  // Button released
    digitalWrite(LED, LOW);
    Serial.println("LED OFF");
  }
  delay(50);  // Debounce delay
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  pinMode(BTN, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);  // Initialize LED as OFF
}

void loop() {
  test_button();
}
