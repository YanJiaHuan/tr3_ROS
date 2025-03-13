#ifndef ELECTROMAGNET_BLUETOOTH_H
#define ELECTROMAGNET_BLUETOOTH_H

#include <Arduino.h>
#include "BluetoothSerial.h" // Library for Classic Bluetooth

// function prototypes
void Electromagnet_bluetooth_setup(); // Setup function for initializing pins and Bluetooth
void Electromagnet_bluetooth_loop();  // Main loop function for controlling electromagnet
#endif