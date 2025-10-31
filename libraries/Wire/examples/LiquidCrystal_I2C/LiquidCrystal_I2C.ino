#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address 0x27, 16 column and 2 rows

void setup() {
  lcd.init();
  lcd.backlight();
}

int counter = 0;
void loop() {
  lcd.clear();
  lcd.setCursor(6, 0);
  lcd.print("ESP32");
  lcd.setCursor(0, 1);
  lcd.print("Show number:" + String(counter));
  counter++;
  delay(2000);
}
