#include <Arduino.h>
#include "Electromagnet_control.h"

// Define GPIO pins
#define BUTTON_SIGNAL_PIN 25          // Button signal pin
#define ELECTROMAGNET_SIGNAL_PIN 27   // Electromagnet signal pin

// Internal state variables
static volatile bool buttonPressed = false;      // Button press flag
static volatile bool electromagnetState = false; // Electromagnet state
// Interrupt Service Routine (ISR) for the button
void IRAM_ATTR Electromagnet_handleButtonPress() {
  buttonPressed = true; // Set the flag when the button is pressed
}

void Electromagnet_control_setup() {
  // Initialize pins
  pinMode(LED_BUILTIN, OUTPUT);             // Set built-in LED as output
  pinMode(BUTTON_SIGNAL_PIN, INPUT_PULLUP); // Set button signal pin as input with pull-up
  // Setup PWM on the ESP32
  ledcSetup(0, 1000, 8);                   // Channel 0, 1kHz frequency, 8-bit resolution
  ledcAttachPin(ELECTROMAGNET_SIGNAL_PIN, 0); // Attach channel 0 to the PWM pin
  
  // Ensure initial states are OFF
  digitalWrite(LED_BUILTIN, LOW);           // Turn off the built-in LED
  ledcWrite(0, 0);    

  
  // Attach interrupt to the button pin
  attachInterrupt(digitalPinToInterrupt(BUTTON_SIGNAL_PIN), Electromagnet_handleButtonPress, FALLING);
}

void Electromagnet_control_loop() {
  // If a button press is detected
  if (buttonPressed) {
    buttonPressed = false;                // Reset the button press flag
    electromagnetState = !electromagnetState; // Toggle the electromagnet state
    
    if (electromagnetState) {
      // Turn ON electromagnet and LED
      digitalWrite(LED_BUILTIN, HIGH);            // Turn on built-in LED
      ledcWrite(0, 255);
    } else {
      // Turn OFF electromagnet and LED
      digitalWrite(LED_BUILTIN, LOW);              // Turn off built-in LED
      ledcWrite(0, 0);
    }
  }
}
