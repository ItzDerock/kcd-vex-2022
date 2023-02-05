# Takes STDIN in format: printf("x, y, angle, reqX, reqY, reqAng, xPow, yPow, angPow\n");
# and plots the data in real time

import sys
import matplotlib.pyplot as plt

# Output data to a file
output = open('./logs/latest.csv', 'w');

# Create three subplots for x, y, and angle
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
fig.suptitle('Real-time PID Loop Tuning')

# Create lists to store data
x_data = []
x_error_data = []
x_power_data = []
y_data = []
y_error_data = []
y_power_data = []
angle_data = []
angle_error_data = []
angle_power_data = []

# Continuously read data from stdin
while True:
    line = sys.stdin.readline()
    if not line:
        break

    # log data to file
    output.write(line)

    # check line length, must be 9 values
    if len(line.split(',')) != 9:
        print("bad line: " + line);
        continue

    # Parse data
    x, y, angle, x_error, y_error, angle_error, x_power, y_power, angle_power = line.split(',')
    
    # If the first value is 'x', it is the header. Skip it
    if x == 'x':
        print("skipping header")
        continue

    x_data.append(float(x))
    x_error_data.append(float(x_error))
    x_power_data.append(float(x_power))
    y_data.append(float(y))
    y_error_data.append(float(y_error))
    y_power_data.append(float(y_power))
    angle_data.append(float(angle))
    angle_error_data.append(float(angle_error))
    angle_power_data.append(float(angle_power))

    # Update x subplot
    ax1.cla()
    ax1.plot(x_data, label='x')
    ax1.plot(x_error_data, label='xError')
    ax1.plot(x_power_data, label='xPower')
    ax1.legend(loc='upper left')

    # Update y subplot
    ax2.cla()
    ax2.plot(y_data, label='y')
    ax2.plot(y_error_data, label='yError')
    ax2.plot(y_power_data, label='yPower')
    ax2.legend(loc='upper left')

    # Update angle subplot
    ax3.cla()
    ax3.plot(angle_data, label='angle')
    ax3.plot(angle_error_data, label='angleError')
    ax3.plot(angle_power_data, label='anglePower')
    ax3.legend(loc='upper left')

    # Update and pause the plot
    plt.pause(0.001)

plt.show()
# Close the file
# output.close()
