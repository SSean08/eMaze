import serial
import csv
import time

# Configure the serial port (change /dev/ttyACM0 to your port, baud rate to match your Arduino)
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
csv_file = 'arduino_data.csv'

time.sleep(2) # Wait for the Arduino to initialize

print("Recording data... Press Ctrl+C to stop.")

try:
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line:
            # Split comma-separated values
            data = line.split(',')
            
            # Save to CSV
            with open(csv_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(data)
            print(f"Logged: {data}")
except KeyboardInterrupt:
    print("Recording stopped.")
    ser.close()

