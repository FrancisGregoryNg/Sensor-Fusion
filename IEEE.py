import csv
'''import pigpio'''
import numpy as np

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------      Initialization     ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Use pigpio daemon to control general-purpose input/outputs more freely
'''pi = pigpio.pi()'''

# Set the size of the room (must be the same with particle_filter.py)
# the room dimensions (width, length) correspond to (x, y)
# the origin starts at a selected corner of the room
# the measurement unit is in millimeters (mm)
room_width = 2000
room_length = 2000

# The balance between precision and difficulty is determined heuristically
# the measurement unit is in millimeters (mm)
grid_precision = 100

# The size is determined inclusive of the origin which is located in a corner
size_of_x = int(1 + np.floor(room_width/grid_precision))
size_of_y = int(1 + np.floor(room_length/grid_precision))

# Calculate the number of datapoints for use in the signal mapping
number_of_datapoints = size_of_x * size_of_y

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------      Grid Locations     ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Set the grid locations
with open('IEEE_matching_locations.csv', 'w') as locations:
    coordinates = ['x', 'y']
    write_locations = csv.DictWriter(locations, fieldnames = coordinates,
                                     lineterminator = '\n')
    write_locations.writeheader()
   
    # Start with the origin
    x, y = 0, 0
    
    # Monitor the reversals because the progression is done in a snake-like manner
    direction = 1 
    counter = 1
    
    # Run the loop to progress through the grid in a snake-like manner
    for run in range(number_of_datapoints):
        write_locations.writerow({'x': x, 'y': y})
        if counter < size_of_x:
            x = x + direction * grid_precision
        else:
            direction = direction * -1
            counter = 0
            y = y + grid_precision
        counter += 1
        
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#--------------------  Synthesize IEEE signal signature  ----------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 
# Define location of IEEE 802.15.4a transmitters
transmitter_0 = (1000, 1000)
transmitter_1 = (2000, 2000)

# Set expected range for signal values (noise could make actual values exceed)
signal_min = 0
signal_max = 255
signal_range = signal_max - signal_min

# Initialize empty arrays
signal_0 = np.zeros((number_of_datapoints, 1), dtype = float)
signal_1 = np.zeros((number_of_datapoints, 1), dtype = float)
x = np.zeros((number_of_datapoints, 1), dtype = float)
y = np.zeros((number_of_datapoints, 1), dtype = float)

# Create an arry of ones for use in computations
ones = np.ones((number_of_datapoints, 1), dtype = float)

# Start with the origin
x_index, y_index = 0, 0
x_value, y_value = 0, 0

# Monitor the reversals because the progression is done in a snake-like manner
direction = 1 
counter = 1

# Run the loop to progress through the grid in a snake-like manner
for run in range(number_of_datapoints):
    (x[x_index], y[y_index]) = (x_value, y_value)
    x_index += 1
    y_index += 1
    if counter < size_of_x:
        x_value = x_value + direction * grid_precision
    else:
        direction = direction * -1
        counter = 0
        y_value = y_value + grid_precision
    counter += 1

# Calculate the signal scale based on the distance from the transmitter
scale_0 = np.sqrt(np.add(np.subtract(x, (transmitter_0[0] * ones)) ** 2, 
                         np.subtract(y, (transmitter_0[1] * ones)) ** 2))
scale_1 = np.sqrt(np.add(np.subtract(x, (transmitter_1[0] * ones)) ** 2,  
                         np.subtract(y, (transmitter_1[1] * ones)) ** 2))

# Normalize the signal scale
scale_0 = scale_0 / np.amax(scale_0)
scale_1 = scale_1 / np.amax(scale_1)

# Shift the signal scale (closer = stronger)
scale_0 = np.amax(scale_0) - scale_0
scale_1 = np.amax(scale_1) - scale_1

# Generate noise
mean = 0
standard_deviation = 10
noise_0 = np.random.normal(mean, standard_deviation, (number_of_datapoints, 1))
noise_1 = np.random.normal(mean, standard_deviation, (number_of_datapoints, 1))

# Calculate the signals
signal_0 = np.sum([(signal_min * ones), (signal_range * scale_0), noise_0], 
                   axis = 0)
signal_1 = np.sum([(signal_min * ones), (signal_range * scale_1), noise_1], 
                   axis = 0)

# Log the data directly onto the .csv file
with open('IEEE_signal_database.csv', 'w') as database:
    sensors = ['IEEE_0', 'IEEE_1', 'x', 'y']
    write_database = csv.DictWriter(database, fieldnames = sensors,
                                    lineterminator = '\n')
    write_database.writeheader()

    for run in range(number_of_datapoints):
        IEEE_0 = float(signal_0[run])
        IEEE_1 = float(signal_1[run])
        x_entry = float(x[run])
        y_entry = float(y[run])
        write_database.writerow({'IEEE_0': IEEE_0, 'IEEE_1': IEEE_1,
                                 'x': x_entry, 'y': y_entry,})
        run = run + 1

'''
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------  IEEE signal signature  ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Open pins for bit bang reading of serial data
pi.bb_serial_read_open(14, 115200)
pi.bb_serial_read_open(15, 115200)

# IEEE 802.15.4a signals are taken before the encoder logging time

# Set the trigger for logging the data
log_data = 0

# Log the data directly onto the .csv file
with open('IEEE_signal_database.csv', 'w') as database:
    sensors = ['IEEE_0', 'IEEE_1']
    write_database = csv.DictWriter(database, fieldnames = sensors)
    write_database.writeheader()

for run in range(number_of_datapoints):
    log_data = 0 # specify pin input in the future
    if log_data == 1:
        run = run + 1
        # Base signal strength on the number of bytes read, ignore bytearray
        (IEEE_0, temp_data_0) = pi.bb_serial_read()
        (IEEE_1, temp_data_1) = pi.bb_serial_read()
        write_database.writerow({'IEEE_0': IEEE_0, 'IEEE_1': IEEE_1})
        
# Close pins for bit bang reading of serial data
pi.bb_serial_read_close(14)
pi.bb_serial_read_close(15)'''