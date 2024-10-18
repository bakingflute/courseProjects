import serial
import matplotlib.pyplot as plt
import time

# Initialize serial connection
# arduinoComPort = 'COM9'  # Change this to your Arduino's port
# baudRate = 9600
# serialPort = serial.Serial(arduinoComPort, baudRate, timeout=1)

# # Collect data from Arduino
datafile = "datafile.txt"

# try:
#     print("Collecting data... Press Ctrl+C to stop.")
#     while True:
#         data = serialPort.readline().decode().strip()

#         if len(data) > 0:  # Check if data was received
#             with open(datafile, "a") as file:  # Open the data file in append mode
#                 file.write(data + '\n')  # Write data
# except Exception:
#     print("Data collection stopped.")
# finally:
#     serialPort.close()  # Ensure the serial port is closed

# # Read data from the file for plotting
leftSensorAdjusted_data = []
rightSensorAdjusted_data = []
leftOutput_data = []
rightOutput_data = []

with open(datafile, "r") as file:
    for line in file:
        print(line)
        values = line.strip().split(',')
        if len(values) == 4:  # Ensure we have the right number of values
            leftSensorAdjusted = 0.7 * float(values[0])
            rightSensorAdjusted = 0.7 * float(values[1])
            leftOutput = float(values[2])
            rightOutput = float(values[3])
            
            # Append to lists
            leftSensorAdjusted_data.append(leftSensorAdjusted)
            rightSensorAdjusted_data.append(rightSensorAdjusted)
            leftOutput_data.append(leftOutput)
            rightOutput_data.append(rightOutput)

# Plotting the data
plt.figure(figsize=(10, 6))

plt.plot(leftSensorAdjusted_data, label='Left Sensor Adjusted', color='r')
plt.plot(rightSensorAdjusted_data, label='Right Sensor Adjusted', color='g')
plt.plot(leftOutput_data, label='Left Output', color='b')
plt.plot(rightOutput_data, label='Right Output', color='m')

plt.xlabel('Samples')
plt.ylabel('Value')
plt.title('Sensor Data and Motor Outputs')
plt.legend()
plt.grid()
plt.show()
