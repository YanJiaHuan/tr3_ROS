# 使用方法：
该项目使用了vscode extension: PlatformIO 作为 IDE, 在此记录一些简单的启动/转换 操作，方便上手

## 单片机型号：
型号：ESP32_devkit_tv1
Built-in blue LED connected to GPIO2; Built-in red LED that shows the board is being powered

## 前序步骤：
### 1. 端口授权
检查自己单片机是通过哪个端口连接的电脑，比如 `ls /dev/tty*`--> `/ttyUSB0`, 可以通过 `chmod 666 /dev/ttyUSB0` 来给这个端口写入的权限，从而才能通过PlatformIO往单片机里烧录程序

## 文件解释

### 1. src
#### 0. src/main.cpp
主要编译，烧录的程序
#### 1. src/Blink.cpp:
实现单片机板载led(默认蓝灯)每隔一秒闪烁一次的功能，在mian.cpp里调用：
```cpp
include "Blink.h"
setup(){Blink_setup}
loop(){Blink_loop}
```
#### 2. src/Blink_v2.cpp
实现外接led灯，每隔1秒点亮，通过板子的GPIO 26 (D26) 接led的长引脚(正极)，GND 接短引脚，然后每隔1秒，往GPIO26通高电平（3.3v），实现亮灭交替，在main.cpp里调用。注意led串限流电阻。
```cpp
include "Blink_v2.h"
setup(){Blink_v2_setup}
loop(){Blink_v2_loop}
```
#### 3. src/Blink_control.cpp
连接方法: 

Joystick:

    Connect the S (signal) pin to GPIO 25 (or any free GPIO pin).
    Connect the VCC pin to the ESP32's 3.3V pin.
    Connect the GND pin to the ESP32's GND pin.

LED:

    Connect the LED's longer leg (anode) to GPIO 26 via a 220-ohm resistor.
    Connect the LED's shorter leg (cathode) to the ESP32's GND pin.
用法：按下按钮，控制外接的led亮或者暗
```cpp
include "Blink_control.h"
setup(){Blink_control_setup}
loop(){Blink_control_loop}
```
#### 4. src/Electromagnet_control.cpp
按键控制电磁铁on/off, 当电磁铁启动时，板子内置的蓝灯也亮起表示电磁铁工作，当电磁铁关闭时，灯灭。
接线：
按键：`+` 接 `VCC`, `-` 接 `GND`, `s-k`接`GPIO 25`
电磁铁：`+`接 `VIN`, `-`接 `GND`, `S` 接 `GPIO 27` (该电磁铁模块需要DC 5V)
```cpp
include "Electromagnet_control.h"
setup(){Electromagnet_control_setup()}
loop(){Electromagnet_control_loop()}
```

#### 5. src/Electromagnet_bluetooth.cpp
蓝牙模块接收端口信息，如果收到1,电磁铁/蓝色led on, 如果收到0，电磁铁/蓝色led off。
接线：
电磁铁：`+`接 `VIN`, `-`接 `GND`, `S` 接 `GPIO 13` (该电磁铁模块需要DC 5V)
配置：
1. esp32烧录程序后通电，会自动开启蓝牙模块
2. 在电脑蓝牙选项里连接该设备
3. (optional) sudo rfcomm bind /dev/rfcomm0 3C:8A:1F:A0:C0:A6
4. 运行 `python Electromagnet_bluetooth_client.py`
```cpp
include "Electromagnet_bluetooth.h"
setup(){Electromagnet_bluetooth_setup()}
loop(){Electromagnet_bluetooth_loop()}
```

### 2. include
#### 1. include/Blink.h
Blink.cpp的头文件
#### 2. include/Blink_v2.h
Blink_v2.cpp的头文件
#### 3. include/Blink_control.h
Blink_control的头文件
#### 4. include/Electromagnet_control.h
Electromagnet_control.cpp的头文件
#### 5. include/Electromagnet_bluetooth.h
Electromagnet_bluetooth.cpp的头文件

### 3. test
#### 1. test/Electromagnet_bluetooth_client.py
用于与单片机(Electromagnet_bluetooth.cpp)通讯的pc端代码，需注意波特率和单片机设置的相同，以及端口名，需要与单片机MAC地址绑定的端口一致