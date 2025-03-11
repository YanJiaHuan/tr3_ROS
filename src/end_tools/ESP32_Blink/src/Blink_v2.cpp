#include <Arduino.h>
#include "Blink_v2.h"

#define LED_PIN 26

void Blink_v2_setup(){
    pinMode(LED_PIN, OUTPUT);
}

void Blink_v2_loop(){
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
}