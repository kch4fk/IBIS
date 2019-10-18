# Kay Hutchinson 10/18/19
# Communicate with and control Arduino

# Import libraries
import serial
import io
import time

# Arduino port and baud rate
port = "/dev/ttyACM0"
baud = 9600
ser = serial.Serial(port,baud)

while (True):

    for i in range(5):

        # Read serial line from Arduino
        x = ser.readline()
        print(x)

    # Write commands to serial (in bytes)
    ser.write(b'I')






# Close the serial property
ser.close()
