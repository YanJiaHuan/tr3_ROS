import serial
import time

# Define the serial port and baud rate
bluetooth_port = "/dev/rfcomm0"
baud_rate = 115200

# Open the serial connection
ser = serial.Serial(bluetooth_port, baud_rate)
time.sleep(2)  # Allow time for the connection to initialize

try:
    while True:
        command = input("Enter '1' to turn ON, '0' to turn OFF (or 'q' to quit): ").strip()
        if command == 'q':
            break
        elif command in ('1', '0'):
            ser.write(command.encode())  # Send the command via Bluetooth
            print(f"Sent: {command}")
        else:
            print("Invalid command. Enter '1', '0', or 'q'.")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    ser.close()
