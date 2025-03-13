#include <Arduino.h>
#include "RelayControl_BLE.h"  // Include the modular header for BLE relay control

void setup() {
  // Initialize the BLE setup function
  RelayControl_BLE_setup();
}

void loop() {
  // Handle BLE communication and control the relay based on BLE messages
  RelayControl_BLE_loop();
}

// void setup() {
//   Serial.begin(115200);
//   delay(1000);
//   Serial.println("Hello, world!");
// }

// void loop() {}
