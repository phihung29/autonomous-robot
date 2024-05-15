import serial
import pynmea2
from datetime import datetime

# Define the serial port and baudrate
serial_port = '/dev/ttyS0'  # Adjust this if your serial port is different
baudrate = 9600  # Adjust this if your GPS module uses a different baudrate

# Initialize variables for storing previous GPS data

prev_time = None

# Open the serial port
ser = serial.Serial(serial_port, baudrate)

try:
    while True:
        # Read a line from the serial port
        line = ser.readline().decode('utf-8')

        # Check if the line contains GGA data
        if line.startswith('$GPGGA'):
            # Parse the NMEA sentence
            msg = pynmea2.parse(line)

            # Extract latitude, longitude, and timestamp from the message
            lat = msg.latitude
            lon = msg.longitude
            #timestamp = datetime.combine(datetime.utcnow().date(), msg.timestamp)
            print(lat,lon)
            # Calculate velocity if previous data exists
    

            # Update previous GPS data
            

finally:
    # Close the serial port
    ser.close()

# Haversine formula to calculate distance between two points given their latitude and longitude

