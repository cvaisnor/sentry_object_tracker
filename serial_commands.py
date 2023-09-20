import serial
import time

# Initialize serial connection (make sure the port and baud rate are the same as you set on the Arduino)
ser = serial.Serial('/dev/ttyACM0', 9600)
print('Serial connection initialized')
time.sleep(3)  # Wait for Arduino to initialize

# function to move X axis
def move_X(degrees):
    steps = degrees * 200 / 360  # Convert degree to steps
    # direction = 1 if steps >= 0 else -1
    steps = abs(int(steps)) # Convert to absolute integer
    ser.write(b'x')  # Send motor ID as bytes
    time.sleep(3)  # Wait for Arduino to process the command
    while ser.in_waiting > 0:
        print(ser.readline().decode().strip())
    
    time.sleep(3)  # Wait for Arduino to process the command


# function to move Y axis
# def move_Y(degree):
#     steps = degree * 200 / 360  # Convert degree to steps
#     direction = 1 if steps >= 0 else -1
#     steps = abs(int(steps)) # Convert to absolute integer
#     ser.write(b'y')  # Send motor ID as bytes
#     command = f'{steps},{direction}\n'  # Format into a command string
#     ser.write(command.encode())  # Send command as bytes
#     while ser.in_waiting > 0:
#         print(ser.readline().decode().strip())




# Call the function
move_X(90)

# Close the serial connection
ser.close()