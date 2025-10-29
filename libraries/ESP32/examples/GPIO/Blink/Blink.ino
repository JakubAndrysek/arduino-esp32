#define LED    4
#define BUTTON 0

uint8_t stateLED = 0;

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
}

void loop() {
  if (!digitalRead(BUTTON)) {
    stateLED = !stateLED;
    digitalWrite(LED, stateLED);
  }
}
