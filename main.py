import serial

# Configure the serial port. Adjust the port and baud rate as needed.
ser = serial.Serial('/dev/ttyUSB0', 9600)

while True:
    # Read a line of data from the Arduino
    data = ser.readline().decode('utf-8').strip()
    
    # Process the received data (parse it into layers and delta_h)
    # For example, split the received string into a list of values
    values = data.split(',')
    if len(values) == 5:
        layers = [float(val) for val in values[:4]]
        delta_h = float(values[4])
        
        # Do something with the received data (print it for now)
        print("Layer Data:", layers)
        print("Delta_h:", delta_h)
