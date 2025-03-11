// RelayControl.h
#ifndef RELAY_CONTROL_H
#define RELAY_CONTROL_H

#include <Arduino.h>

// Pin definitions
#define BUTTON_PIN 9  // boot键IO
#define RELAY_PIN 4   // 继电器IO口
#define DEBOUNCE_DELAY 50  // 去抖动延时，单位：毫秒  

// Function prototypes
void RelayControl_setup();
void RelayControl_loop();

#endif
