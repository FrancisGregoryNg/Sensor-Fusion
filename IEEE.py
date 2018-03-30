import csv
import pigpio
import numpy as np

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------      Initialization     ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Use pigpio daemon to control general-purpose input/outputs more freely
pi = pigpio.pi()

# Set the size of the room (must be the same with particle_filter.py)
# the room dimensions (width, length) correspond to (x, y)
# the origin starts at a selected corner of the room
# the measurement unit is in millimeters (mm)
room_width = 5000
room_length = 5000

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

# Start with the origin
x, y = 0, 0

# Monitor the reversals because the progression is done in a snake-like manner
direction = 1
counter = 0

# Set the grid locations
with open('IEEE_matching_locations.csv', 'w') as locations:
    coordinates = ['x', 'y']
    write_locations = csv.DictWriter(locations, fieldnames = coordinates)
    write_locations.writeheader()

# Make the grid locations using snake-like progression
for run in range(number_of_datapoints):
    if counter < size_of_x:
        x = x + direction
    else:
        direction = direction * -1
        x = x + direction
        y = y + 1
    write_locations.writerow({'x': x, 'y': y})


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
pi.bb_serial_read_close(15)