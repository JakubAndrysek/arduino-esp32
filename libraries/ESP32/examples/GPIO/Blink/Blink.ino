// Define pin numbers
#define LED_PIN    4
#define BUTTON_PIN 0

void setup() {
  // Initialize the LED pin as an output
  pinMode(LED_PIN, OUTPUT);

  // Initialize the button pin as an input with internal pull-up resistor
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
  // Check if button is pressed (LOW because of INPUT_PULLUP)
  if (digitalRead(BUTTON_PIN) == LOW) {
    // Blink LED while button is held down
    digitalWrite(LED_PIN, HIGH);  // Turn LED on
    delay(100);                   // Wait 100ms
    digitalWrite(LED_PIN, LOW);   // Turn LED off
    delay(100);                   // Wait 100ms
  } else {
    // Keep LED off when button is not pressed
    digitalWrite(LED_PIN, LOW);
  }
}
