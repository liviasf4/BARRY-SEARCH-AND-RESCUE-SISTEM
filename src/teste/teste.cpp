#include <Arduino.h>

void setup() {
  Serial2.begin(115200, SERIAL_8N1, 26, 25); // RX, TX
}

void loop() {
  Serial2.println("temp:25.4");
  delay(1000);
}