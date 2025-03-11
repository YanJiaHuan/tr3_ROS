// RelayControl.cpp
#include "RelayControl.h"

// Variable definitions
unsigned long lastDebounceTime = 0;  // 上次检查去抖动的时间  
bool lastButtonState = HIGH;         // 上次读取的按钮状态  
bool relayState = false;             // 继电器的当前状态  

void RelayControl_setup() {  
  pinMode(BUTTON_PIN, INPUT_PULLUP); // 设置按钮引脚为输入模式，并启用内部上拉电阻  
  pinMode(RELAY_PIN, OUTPUT);        // 设置继电器引脚为输出模式  
  Serial.begin(115200);              // 初始化串口通信  
}  

void RelayControl_loop() {  
  unsigned long currentTime = millis(); // 获取当前时间  

  // 读取按钮状态  
  int reading = digitalRead(BUTTON_PIN);  

  // 检查是否需要去抖动  
  if ((currentTime - lastDebounceTime) > DEBOUNCE_DELAY) {  
    // 如果按钮状态与上次不同  
    if (reading != lastButtonState) {  
      // 重置去抖动时间  
      lastDebounceTime = currentTime;  

      // 更新最后按钮状态  
      lastButtonState = reading;  

      // 如果按钮被按下（内部上拉，所以读到LOW是按下）  
      if (reading == LOW) {  
        // 切换继电器状态  
        relayState = !relayState;  

        // 根据继电器状态设置继电器引脚  
        digitalWrite(RELAY_PIN, relayState);  

        // 打印继电器状态  
        if (relayState) {  
          Serial.println("Relay ON");  
        } else {  
          Serial.println("Relay OFF");  
        }  
      }  
    }  
  }  
}
