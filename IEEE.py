import csv
import pigpio
import numpy as np

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------      Initialization     ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Use pigpio daemon to control general-purpose input/outputs more freely
pi = pigpio.pi()

# For TX of IEEE_0: (board 16, BCM 23)
# For TX of IEEE_1: (board 18, BCM 24)
IEEE_0_TX = 1
IEEE_1_TX = 1

# For RX of IEEE_0: (board 16, BCM 23)
# For RX of IEEE_1: (board 18, BCM 24)
IEEE_0_RX = 16
IEEE_1_RX = 18
baud = 9600

pi.set_mode(IEEE_0_TX, pigpio.OUTPUT)
pi.set_mode(IEEE_1_TX, pigpio.OUTPUT)

# CLose both RX pins if open
# Exceptions is turned off to prevent error in closing unopened GPIO
pigpio.exceptions = False
pi.bb_serial_read_close(IEEE_0_RX)
pi.bb_serial_read_close(IEEE_1_RX)
pigpio.exceptions = True

# Set up the commands for getting the signal strength from the transmitters
pi.wave_clear()
command = 'DB'
pi.wave_add_serial(IEEE_0_TX, baud, command)
get_strength_0 = pi.wave_create()
pi.wave_add_serial(IEEE_1_TX, baud, command)
get_strength_1 = pi.wave_create()

# Open pins for reading of serial data
pi.bb_serial_read_open(IEEE_0_RX, baud)
pi.bb_serial_read_open(IEEE_1_RX, baud)

# Set pin (board 36, BCM 16) as input for start button 
start_button = 16
# This may be used to trigger data-gathering, or to resume after a stop
pi.set_mode(start_button, pigpio.INPUT)

# Set pin (board 38, BCM 20) as input for stop button
stop_button = 20
# This may be used to stop data-gathering, or to end the program after a stop.
pi.set_mode(stop_button, pigpio.INPUT)

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
    start = pi.read(start_button) 
    stop = pi.read(stop_button)

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
                start = pi.read(start_button) 
                stop = pi.read(stop_button)
                
                # Gather data for every press of the start button
                if (start == 1):
                    start = 0

                    # Request signal strength from the first transmitter
                    pi.wave_send_once(get_strength_0)
                    
                    # Wait until all data has been sent
                    while pi.wave_tx_busy():
                          pass
                    
                    # Read signal strength data from the first transmitter
                    (temp_data_0, IEEE_0) = pi.bb_serial_read(IEEE_0_RX)
                    
                    # Request signal strength from the second transmitter
                    pi.wave_send_once(get_strength_1)
                    
                    # Wait until all data has been sent
                    while pi.wave_tx_busy():
                          pass
                    
                    # Read signal strength data from the second transmitter
                    (temp_data_1, IEEE_1) = pi.bb_serial_read(IEEE_1_RX)
                    
                    # Convert bytearray to integer 
                    IEEE_0 = int.from_bytes(IEEE_0)  
                    IEEE_1 = int.from_bytes(IEEE_1)  
    
    # Reset the stop button
    # Wait for user to confirm that the program will terminate                
    stop = 0
    
# Close pigpio
pi.wave_delete(0)
pi.wave_delete(1)
pi.bb_serial_read_close(IEEE_0_RX)
pi.bb_serial_read_close(IEEE_1_RX)
pi.stop()
'''
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#--------------------  Synthesize IEEE 802.15.4a signals  ----------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#------------------------------------------------------------------------------
# Set up database
#------------------------------------------------------------------------------

# Initialize the corresponding location matrix
location = np.zeros((1,2), dtype = float)
    
# Copy data from the .csv file onto the corresponding matrix
with open('IEEE_matching_locations.csv') as matching_locations:
    read_locations = csv.DictReader(matching_locations)
    for row in read_locations:
        location = np.append(location, 
                              [[float(row['x']), float(row['y'])]], 
                              axis = 0)
        
# Delete the initial zero-value rows of the matrices
location = np.delete(location, (0), axis = 0)
    
# the room dimensions (width, length) correspond to (x, y)
# the origin starts at a selected corner of the room
# the measurement unit is in millimeters (mm)
room_width = 2000
room_length = 2000
    
# Define location of IEEE 802.15.4a transmitters
transmitter_0 = (0, 0)
transmitter_1 = (2000, 0)

number_of_datapoints = location.shape[0]

# Initialize empty arrays
signal_0 = np.zeros((number_of_datapoints, 1), dtype = float)
signal_1 = np.zeros((number_of_datapoints, 1), dtype = float)

x = location[:,[0]]
y = location[:,[1]]

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

# Calculate the factor for the inverse-square law
factor_0 = signal_range * np.amin(distance_0[np.nonzero(distance_0)]) ** 2
factor_1 = signal_range * np.amin(distance_1[np.nonzero(distance_1)]) ** 2

# signal_strength = factor * (1 / distance ** 2)
signal_0 = np.divide((factor_0 * ones), np.square(distance_0))
signal_1 = np.divide((factor_1 * ones), np.square(distance_1))

# Generate noise
mean = 0
standard_deviation = 10
noise_0 = np.random.normal(mean, standard_deviation, (number_of_datapoints, 1))
noise_1 = np.random.normal(mean, standard_deviation, (number_of_datapoints, 1))

# Calculate to match signal range, then add noise
signal_0 = np.sum([signal_0, (signal_min * ones), noise_0], axis = 0)
signal_1 = np.sum([signal_1, (signal_min * ones), noise_1], axis = 0)

# Squish the values of the signals to fit the range
signal_0[np.isinf(signal_0)] = signal_max
signal_0 = ((signal_0 - np.amin(signal_0) * ones)
            * signal_max / (np.amax(signal_0) - np.amin(signal_0)))
signal_1[np.isinf(signal_1)] = signal_max
signal_1 = ((signal_1 - np.amin(signal_1) * ones)
            * signal_max / (np.amax(signal_1) - np.amin(signal_1)))

# Log the data directly onto the .csv file* 
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
'''