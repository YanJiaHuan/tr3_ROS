#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "RelayControl_BLE.h"

#define DEVICE_NAME "ESP32_RELAY_CONTROL"  // BLE device name
#define RELAY_PIN 4  // Define the pin for relay (update accordingly)

// UUIDs for Service and Characteristic
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-abcdef987654"

BLECharacteristic *pCharacteristic;
BLEServer *pServer;

static bool relayState = false;

void RelayControl_BLE_setup() {
    Serial.begin(115200);
    delay(2000);  // Short delay to ensure Serial is ready
    Serial.println("Starting setup...");

    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);  // Start with relay OFF

    // Initialize BLE
    BLEDevice::init(DEVICE_NAME);
    pServer = BLEDevice::createServer();

    // Create BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create BLE Characteristic
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
    );
    pCharacteristic->setValue("0");  // Initial value: OFF

    pService->start();  // Start the service

    // Configure Advertising
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);  // Include Service UUID in advertising
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // Helps with connectivity on iOS/Android
    pAdvertising->start();

    // Print MAC address
    std::string macAddress = BLEDevice::getAddress().toString();
    Serial.print("Device MAC Address: ");
    Serial.println(macAddress.c_str());  // Print the MAC address as a C-string

    Serial.println("BLE Advertising Started. Device name: " DEVICE_NAME);
}

void RelayControl_BLE_loop() {
    // Restart advertising if it is not active
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();  // Manually start advertising if it stopped

    String command = pCharacteristic->getValue().c_str();

    if (command == "1" && !relayState) {
        relayState = true;
        digitalWrite(RELAY_PIN, HIGH);
        Serial.println("Relay ON");
        pCharacteristic->setValue("1");
    } else if (command == "0" && relayState) {
        relayState = false;
        digitalWrite(RELAY_PIN, LOW);
        Serial.println("Relay OFF");
        pCharacteristic->setValue("0");
    }
    delay(10);  // Small delay to prevent loop from hogging CPU
}
