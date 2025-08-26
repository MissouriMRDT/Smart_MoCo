#include <Arduino.h>

void setup() { pinMode(PA4, OUTPUT); }

void loop() {
  digitalWrite(PA4, LOW);
  delay(1000);
  digitalWrite(PA4, HIGH);
  delay(1000);
}