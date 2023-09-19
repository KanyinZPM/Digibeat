import serial
import csv
import time

# Configure the UART port
ser = serial.Serial('COM5', baudrate=57600)  # Replace 'COM5' with your UART port

# Open the CSV file for writing
with open('ecg_data.csv', 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)

    # Write header row
    header = ['Timestamp', 'Heart Reading']
    csvwriter.writerow(header)

    # Read and write UART data to CSV
    try:
        while True:
            uart_data = ser.readline().decode().strip()  # Read data from UART and decode
            print(uart_data)  # Print data to console
            timestamp = time.strftime('%Y-%m-%d %H:%M:%S')  
            csvwriter.writerow([time.time(), uart_data])  # Write data to CSV with timestamp
    except KeyboardInterrupt:
        ser.close()


