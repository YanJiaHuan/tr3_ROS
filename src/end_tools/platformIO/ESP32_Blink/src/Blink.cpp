#include <Arduino.h>
#include "Blink.h"
// put function declarations here:
// int myFunction(int, int);

void Blink_setup() {
  // put your setup code here, to run once:
  // int result = myFunction(2, 3);
  pinMode(LED_BUILTIN, OUTPUT);

}

void Blink_loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
}

// put function definitions here:
// int myFunction(int x, int y) {
//   return x + y;
// }