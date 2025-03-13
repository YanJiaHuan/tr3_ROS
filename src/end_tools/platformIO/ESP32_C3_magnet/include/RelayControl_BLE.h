// RelayControl_BLE.h
#ifndef RELAY_CONTROL_BLE_H
#define RELAY_CONTROL_BLE_H

#include <Arduino.h>

// Pin definitions
#define RELAY_PIN 4   // Relay control pin for electromagnet

// Function prototypes for the BLE setup and loop functions
void RelayControl_BLE_setup();    // Setup the BLE server and start advertising
void RelayControl_BLE_loop();     // Handle BLE communication and relay control

#endif  // RELAY_CONTROL_BLE_H
