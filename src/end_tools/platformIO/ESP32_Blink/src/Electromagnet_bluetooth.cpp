#include <Arduino.h>
#include "BluetoothSerial.h" // Library for Classic Bluetooth
#include "Electromagnet_bluetooth.h"

// Define GPIO pins
#define ELECTROMAGNET_SIGNAL_PIN 27 // Electromagnet signal pin

// Create a BluetoothSerial object
BluetoothSerial SerialBT;

// Internal state variable for electromagnet state
static bool electromagnetState = false;

void Electromagnet_bluetooth_setup() {
  // Initialize Serial communication for debugging
  Serial.begin(115200);
  
  // Initialize Bluetooth
  SerialBT.begin("ESP32_Electromagnet"); // Bluetooth device name
  String macAddress = SerialBT.getBtAddressString();
  Serial.print("Bluetooth MAC Address: ");
  Serial.println(macAddress);

  // Initialize pins
  pinMode(LED_BUILTIN, OUTPUT); // Set built-in LED as output
  // Setup PWM on the ESP32
  ledcSetup(0, 1000, 8);                   // Channel 0, 1kHz frequency, 8-bit resolution
  ledcAttachPin(ELECTROMAGNET_SIGNAL_PIN, 0); // Attach channel 0 to the PWM pin

  // Ensure initial states are OFF
  digitalWrite(LED_BUILTIN, LOW); // Turn off the built-in LED
  ledcWrite(0, 0);                // Turn off the electromagnet
}

void Electromagnet_bluetooth_loop() {
  // Check if there is data available on Bluetooth
  if (SerialBT.available()) {
    char receivedChar = SerialBT.read(); // Read the incoming data

    if (receivedChar == '1') {
      // Turn ON electromagnet and LED
      electromagnetState = true;
      digitalWrite(LED_BUILTIN, HIGH); // Turn on built-in LED
      ledcWrite(0, 255);              // Turn on electromagnet
      Serial.println("Electromagnet ON");
    } else if (receivedChar == '0') {
      // Turn OFF electromagnet and LED
      electromagnetState = false;
      digitalWrite(LED_BUILTIN, LOW); // Turn off built-in LED
      ledcWrite(0, 0);               // Turn off electromagnet
      Serial.println("Electromagnet OFF");
    }
  }
}
