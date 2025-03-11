from bluepy.btle import Peripheral, UUID, Characteristic
import time
import sys

# Replace with your ESP32-C3 BLE device MAC address
device_mac = "28:37:2f:48:ee:ee"  # Replace with the actual MAC address of your ESP32-C3

# Define the service and characteristic UUIDs (same as defined in ESP32)
SERVICE_UUID = UUID("12345678-1234-1234-1234-123456789abc")
CHARACTERISTIC_UUID = UUID("87654321-4321-4321-4321-abcdef987654")

def connect_device(mac_address):
    try:
        print(f"Trying to connect to {mac_address}...")
        device = Peripheral(mac_address)
        print("Connected successfully!")
        return device
    except Exception as e:
        print(f"Error connecting to device: {e}")
        return None

def relay_control(device):
    service = device.getServiceByUUID(SERVICE_UUID)
    characteristic = service.getCharacteristics(CHARACTERISTIC_UUID)[0]
    
    try:
        while True:
            command = input("Enter '1' to turn ON, '0' to turn OFF (or 'q' to quit): ").strip()
            if command == 'q':
                break
            elif command in ('1', '0'):
                characteristic.write(command.encode())  # Send command via BLE
                print(f"Sent: {command}")
            else:
                print("Invalid command. Enter '1', '0', or 'q'.")
    except KeyboardInterrupt:
        print("\nDisconnected by user.")
    finally:
        device.disconnect()
        print("Disconnected from device.")

def main():
    device = None

    while device is None:
        device = connect_device(device_mac)
        if device is None:
            print("Retrying in 5 seconds...")
            time.sleep(1)

    relay_control(device)

if __name__ == "__main__":
    main()
