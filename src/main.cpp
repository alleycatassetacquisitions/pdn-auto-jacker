#include "MotorController.h"

// Create the MotorController instance
// MotorController motorController(Serial, Serial1);

void setup() {
  Serial.begin(115200);
    // motorController.setup();
}

void loop() {
  if (Serial2.available()) {
    Serial.println(Serial2.readStringUntil('\n'));
  }
    // motorController.loop();
}