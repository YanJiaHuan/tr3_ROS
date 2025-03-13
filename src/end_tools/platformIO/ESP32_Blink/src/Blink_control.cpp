// #include <Arduino.h>
// #include "Blink_control.h"

// // Define GPIO pins
// #define LED_PIN 26         // GPIO for the LED
// #define BUTTON_PIN 25      // GPIO for the joystick's push button (signal pin)

// // Internal state variables
// static bool ledState = false;      // Keep track of LED state


// void Blink_control_setup() {
//   pinMode(LED_PIN, OUTPUT);         // Set LED pin as output
//   pinMode(BUTTON_PIN, INPUT_PULLUP); // Set button pin as input with pull-up resistor
//   digitalWrite(LED_PIN, HIGH);       // Ensure LED starts in the OFF state
// }

// void Blink_control_loop() {
//   // Read the current button state
//   int currentButtonState = digitalRead(BUTTON_PIN);
//   if (currentButtonState == LOW){
//     ledState = !ledState;                  // Toggle the LED state
//     digitalWrite(LED_PIN, ledState ? HIGH : LOW); 
//   }

// }
#include <Arduino.h>
#include "Blink_control.h"

// Define GPIO pins
#define LED_PIN 26         // GPIO for the LED
#define BUTTON_PIN 25      // GPIO for the joystick's push button (signal pin)

// Internal state variables
static volatile bool ledState = false;    // Keep track of LED state
static volatile bool buttonPressed = false; // Button press flag

// Interrupt Service Routine (ISR)
void IRAM_ATTR handleButtonPress() {
  buttonPressed = true; // Set the flag when the button is pressed
}

void Blink_control_setup() {
  pinMode(LED_PIN, OUTPUT);         // Set LED pin as output
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Set button pin as input with pull-up resistor
  digitalWrite(LED_PIN, HIGH);       // Ensure LED starts in the OFF state
  
  // Attach interrupt to button pin on falling edge (button pressed)
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING);
}

void Blink_control_loop() {
  // If a button press is detected, toggle the LED state
  if (buttonPressed) {
    buttonPressed = false;         // Reset the button press flag
    ledState = !ledState;          // Toggle the LED state
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  }
}

