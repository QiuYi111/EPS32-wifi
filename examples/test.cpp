#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(1);
  Serial.println("Test setup complete");
}
void loop() {
    Serial.println("Test loop running");
    delay(1000);    
  // Empty loop
}