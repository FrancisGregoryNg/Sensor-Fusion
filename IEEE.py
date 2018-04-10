import csv
import pigpio
import numpy as np

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------      Initialization     ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Use pigpio daemon to control general-purpose input/outputs more freely
pi = pigpio.pi()

# Set pin (board 36, BCM 16) as input for start button 
# This may be used to trigger data-gathering, or to resume after a stop
pi.set_mode( 16, pigpio.INPUT)

# Set pin (board 38, BCM 20) as input for stop button
# This may be used to stop data-gathering, or to end the program after a stop.
pi.set_mode( 20, pigpio.INPUT)

# Set buttons to unactivated status
start = 0
stop = 0

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-----------------      Gather IEEE 802.15.4a signals     ---------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Write the header for the .csv file
with open('IEEE_signal_database.csv', 'w') as database:
    sensors = ['IEEE_0', 'IEEE_1']
    write_database = csv.DictWriter(database, fieldnames = sensors,
                                    lineterminator = '\n')
    write_database.writeheader()

# Continuously wait for change in button activation
# Either start data-gathering mode or terminate the program       
while (stop == 0):
    start = pi.read(16) 
    stop = pi.read(20)

    # Start the data-gathering mode
    if (start == 1):
        
        # Reset the button values
        start = 0
        stop = 0
        
        # Stay in data-gathering mode until the stop button is pressed
        while (stop == 0):
            # Access the .csv file
            with open('IEEE_signal_database.csv', 'a') as database:
                sensors = ['IEEE_0', 'IEEE_1']
                write_database = csv.DictWriter(database, fieldnames = sensors,
                                                lineterminator = '\n')
                # Wait for button activation
                start = pi.read(16) 
                stop = pi.read(20)
                
                # Gather data for every press of the start button
                if (start == 1):
                    start = 0

                    # Open pins for bit bang reading of serial data
                    pi.bb_serial_read_open(14, 115200)
                    pi.bb_serial_read_open(15, 115200)
                    
                    # Base signal strength on the number of bytes read
                    # Ignore the bytearray
                    (IEEE_0, temp_data_0) = pi.bb_serial_read(14)
                    (IEEE_1, temp_data_1) = pi.bb_serial_read(15)
                    write_database.writerow({'IEEE_0': IEEE_0,
                                             'IEEE_1': IEEE_1})
        
                    # Close pins for bit bang reading of serial data
                    pi.bb_serial_read_close(14)
                    pi.bb_serial_read_close(15)
    
    # Reset the stop button
    # Wait for user to confirm that the program will terminate                
    stop = 0

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#--------------------  Synthesize IEEE 802.15.4a signals  ----------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 
# the room dimensions (width, length) correspond to (x, y)
# the origin starts at a selected corner of the room
# the measurement unit is in millimeters (mm)
room_width = 2000
room_length = 2000
    
# Define location of IEEE 802.15.4a transmitters
transmitter_0 = (1000, 1000)
transmitter_1 = (2000, 2000)

# Set the number of datapoints
number_of_datapoints = 100

# Initialize empty arrays
signal_0 = np.zeros((number_of_datapoints, 1), dtype = float)
signal_1 = np.zeros((number_of_datapoints, 1), dtype = float)

# Distribute the datapoints according to a uniform distribution
x = np.random.uniform(0, room_width, (number_of_datapoints, 1))
y = np.random.uniform(0, room_length, (number_of_datapoints, 1))

# Create an arry of ones for use in computations
ones = np.ones((number_of_datapoints, 1), dtype = float)

# Calculate the distances from the transmitters
distance_0 = np.sqrt(np.add(np.power(np.subtract(x, (transmitter_0[0] * ones)),
                                     2),
                            np.power(np.subtract(y, (transmitter_0[1] * ones)),
                                     2)))
distance_1 = np.sqrt(np.add(np.power(np.subtract(x, (transmitter_1[0] * ones)),
                                     2),
                            np.power(np.subtract(y, (transmitter_1[1] * ones)),
                                     2)))

# Set expected range for signal values (noise could make actual values exceed)
signal_min = 0
signal_max = 255
signal_range = signal_max - signal_min

# Calculate the factor for the inverse-square law=
factor_0 = signal_range * np.amin(distance_0) ** 2
factor_1 = signal_range * np.amin(distance_1) ** 2

# signal_strength = factor * (1 / distance ** 2)
signal_0 = np.divide((factor_0 * ones), np.square(distance_0))
signal_1 = np.divide((factor_1 * ones), np.square(distance_1))

# Generate noise
mean = 0
standard_deviation = 10
noise_0 = np.random.normal(mean, standard_deviation, (number_of_datapoints, 1))
noise_1 = np.random.normal(mean, standard_deviation, (number_of_datapoints, 1))

# Calculate to match signal range, then add noise (signals can exceed range)
signal_0 = np.sum([signal_0, (signal_min * ones), noise_0], axis = 0)
signal_1 = np.sum([signal_1, (signal_min * ones), noise_1], axis = 0)

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