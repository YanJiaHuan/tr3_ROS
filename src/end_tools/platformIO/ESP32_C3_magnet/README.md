# 使用方法：
该项目使用了vscode extension: PlatformIO 作为 IDE, 在此记录一些简单的启动/转换 操作，方便上手

## 单片机型号：
型号：基于esp32-c3-devkitm-1的厂家自研板子，搭载了一个单路继电器（IO口是4），自带led小灯（和继电器直接连接）和按钮(boot键，IO口是9)。

## 前序步骤：
### 1. 端口授权
检查自己单片机是通过哪个端口连接的电脑，比如 `ls /dev/tty*`--> `/ttyACM0`, 可以通过 `chmod 666 /dev/ttyACM0` 来给这个端口写入的权限，从而才能通过PlatformIO往单片机里烧录程序

## 文件解释

### 1. src
#### 0. src/main.cpp
主要编译，烧录的程序

#### 1. src/RelayControl.cpp:
Implementation of relay control, where the button press toggles the relay state:
- Button pin: GPIO 9
- Relay pin: GPIO 4
The electromagnet is controlled via a relay connected to the ESP32 GPIO pins.

#### 2. src/RelayControl_BLE.cpp:
因为esp32c3上没有传统蓝牙模块，只有BLE（低功耗蓝牙模块），这个代码可以开启单片机的BLE广播，并往串口里发MAC地址，client端代码通过这个MAC地址去连单片机的蓝牙
### 2. include
#### 1. include/RelayControl.h
RelayControl.cpp的头文件
#### 2. src/RelayControl_BLE.h
RelayControl_BLE.cpp的头文件

### 3. test
#### 1. test/test_RelayControl_BLE.py
一个示例client端代码，可以通过BLE通信，控制继电器的开关，从而控制电磁铁的开关。优化了相关代码，让程序退出时，启动BLE （因为中断连接会导致BLE关闭），使得单片机的蓝牙还能被找到。