import numpy as np
from numpy.random import uniform
import csv
import pigpio
import time

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------   Function Definitions  ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Used for getting the position using the three encoder values
def get_velocity_encoder(encoder_0, encoder_1, encoder_2):
    # For a delta configuration of the omni-wheels /_\
    # (\) Right is encoder_0, 
    # (/) Left is encoder_1, 
    # (_) Bottom is encoder_2
    # Treat direction along clockwise rotation as positive
    x = (0.5 * encoder_0 + -0.5 * encoder_1 + encoder_2) / 3
    y = (0.57735 * encoder_0 + 0.57735 * encoder_1) / 2
    return x, y

# Used for getting the index of the closes pairs of values in a nx2 matrix
def bestfit(matrix, value_0, value_1):
    value = (value_0, value_1)
    index = ((x ** 2 + y ** 2) for (x, y) in (matrix-value)).argmin
    return index

# Used for getting the position using the IEEE 802.15.4a values
def get_position_IEEE(IEEE_0_start, IEEE_1_start, IEEE_0_end, IEEE_1_end):
    # find the most similar signal signature from database, then get the mean
    database
    coordinates_start = bestfit(database, IEEE_0_start, IEEE_1_start)
    coordinates_end = bestfit(database, IEEE_0_end, IEEE_1_end)
    (x_start, y_start) = locations(coordinates_start)
    (x_end, y_end) = locations(coordinates_end)
    x = (x_start + x_end) / 2
    y = (y_start + y_end) / 2
    return x, y

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------      Initialization     ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#------------------------------------------------------------------------------
# Configure the pin connections
#------------------------------------------------------------------------------

# Use pigpio daemon to control general-purpose input/outputs more freely
pi = pigpio.pi()

# Set sensor pin inputs for the encoders

# Set pin A of encoder_0 (board 29, BCM 5) as input
pi.set_mode( 5, pigpio.INPUT)
# Set pin B of encoder_0 (board 31, BCM 6) as input
pi.set_mode( 6, pigpio.INPUT)

# Set pin A of encoder_1 (board 32, BCM 12) as input
pi.set_mode( 12, pigpio.INPUT)
# Set pin B of encoder_1 (board 33, BCM 13) as input
pi.set_mode( 13, pigpio.INPUT)

# Set pin A of encoder_2 (board 35, BCM 19) as input
pi.set_mode( 19, pigpio.INPUT)
# Set pin B of encoder_2 (board 37, BCM 26) as input
pi.set_mode( 26, pigpio.INPUT)

# Define the sequence of encoder values (cycles every 2 degrees of rotation)
sequence = ((0,0), (0,1), (1,1), (1,0))

#------------------------------------------------------------------------------
# Set up database
#------------------------------------------------------------------------------

# Initialize the signal mapping matrix and the corresponding location matrix
signal_map = np.zeros((1,2), dtype = float)
locations = np.zeros((1,2), dtype = float)
    
# Copy data from the .csv files onto the corresponding matrices
with open('IEEE_signal_database.csv') as database:
    read_database = csv.DictReader(database)
    for row in read_database:
        signal_map = np.append([[row['IEEE_0'], row['IEEE_1']]], axis = 0)
       
with open('IEEE_matching_locations.csv') as locations:
    read_locations = csv.DictReader(locations)
    for row in read_locations:
        locations = np.append([[row['x'], row['y']]], axis = 0)
        
# Delete the initial zero-value rows of the matrices
signal_map = np.delete(signal_map, (0), axis = 0)
locations = np.delete(locations, (0), axis = 0)

#------------------------------------------------------------------------------
# Set values
#------------------------------------------------------------------------------

# number of particles is set heuristically
number_of_particles = 1000

# the room dimensions (width, length) correspond to (x, y)
# the origin starts at a selected corner of the room
# the measurement unit is in millimeters (mm)
room_width = 5000
room_length = 5000

# the maximum speed limit of the AGV is estimated (the lower the better)
# the measurement unit is in millimeters per second (mm/s)
max_speed = 1000

# the frequency of sensor input is identified (unified for both sensors)
# the measurement unit is in hertz
sensor_frequency = 60

#------------------------------------------------------------------------------
# Draw samples from a uniform distribution (initial distribution)
#------------------------------------------------------------------------------

# Initialize the state matrix with zeros
state_matrix = np.zeros((number_of_particles, 4), dtype = float)

# Fill state date for each particle
for particle in range(number_of_particles):
    
    #the state matrix is composed of the state vectors of all particles
    for state_vector_entry in range(4):
        
        # state_vector_0 = x-position from IEEE 802.15.4a input
        state_matrix[particle][0] = uniform(low = 0, high = room_width)
        
        # state_vector_1 = y-position from IEEE 802.15.4a input
        state_matrix[particle][1] = uniform(low = 0, high = room_length)
        
        # state_vector_2 = x-velocity from encoder input
        state_matrix[particle][2] = uniform(low = -max_speed, high = max_speed)
        
         # state_vector_3 = y-velocity from encoder input
        state_matrix[particle][3] = uniform(low = -max_speed, high = max_speed)

#------------------------------------------------------------------------------
# Gather sensor values
#------------------------------------------------------------------------------

# Open pins for bit bang reading of serial data
pi.bb_serial_read_open(14, 115200)
pi.bb_serial_read_open(15, 115200)

# IEEE 802.15.4a signals are taken before the encoder logging time
# Base signal strength on the number of bytes read, ignore bytearray
(IEEE_0_start, temp_data_0) = pi.bb_serial_read()
(IEEE_1_start, temp_data_1) = pi.bb_serial_read()

# Close pins for bit bang reading of serial data
pi.bb_serial_read_close(14)
pi.bb_serial_read_close(15)

# Set time for counter encoder rotations to 1 second
timeout = time.time()

# Initialize the encoder counter
encoder_0 = 0
encoder_1 = 0
encoder_2 = 0

# Take the initial measurements
current_0 = sequence.index((pi.read(5), pi.read(6))) 
current_1 = sequence.index((pi.read(12), pi.read(13))) 
current_2 = sequence.index((pi.read(19), pi.read(26))) 

# Loop until timeout occurs
# Measurement is assumed to be fast enough to limit increments to only 1 or -1
while time.time() < timeout:
    
    # Consider previous measurements
    previous_0 = current_0
    previous_1 = current_1
    previous_2 = current_2
    
    # Update the current measurements
    current_0 = sequence.index((pi.read(5), pi.read(6))) 
    current_1 = sequence.index((pi.read(12), pi.read(13))) 
    current_2 = sequence.index((pi.read(19), pi.read(26)))
    
    # Update the current position
    (encoder_0_A, encoder_0_B) = sequence(current_0)
    (encoder_0_A, encoder_0_B) = sequence(current_1)
    (encoder_0_A, encoder_0_B) = sequence(current_2)
    
    # Increment the encoder value by difference of the two positions
    increment_0 = (current_0 - previous_0)
    increment_1 = (current_1 - previous_1)
    increment_2 = (current_2 - previous_2)
    
    # Value is wrong by a factor of -1/3 upon traversing the sequence endpoint
    if increment_0 != (1 or -1):
        increment_0 == increment_0 * (-1/3)
    if increment_1 != (1 or -1):
        increment_1 == increment_1 * (-1/3)
    if increment_2 != (1 or -1):
        increment_2 == increment_2 * (-1/3)
        
    # Update the encoder counter
    encoder_0 += increment_0
    encoder_1 += increment_1
    encoder_2 += increment_2

# Open pins for bit bang reading of serial data
pi.bb_serial_read_open(14, 115200)
pi.bb_serial_read_open(15, 115200)

# IEEE 802.15.4a signals are retaken after the encoder logging time
# Base the signal strength on the number of bytes read, ignoring bytearray
(IEEE_0_end, temp_data_0) = pi.bb_serial_read()
(IEEE_1_end, temp_data_1) = pi.bb_serial_read()

# Close pins for bit bang reading of serial data
pi.bb_serial_read_close(14)
pi.bb_serial_read_close(15)

# from IEEE 802.15.4a input
get_position_IEEE(IEEE_0_start, IEEE_1_start, IEEE_0_end, IEEE_1_end)

# from encoder input
velocity_x, velocity_y = get_velocity_encoder(encoder_0, encoder_1, encoder_2)

#------------------------------------------------------------------------------
# Compute the weights
#------------------------------------------------------------------------------



#------------------------------------------------------------------------------
# Normalize the weights
#------------------------------------------------------------------------------



#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------  Main Loop (Iterations) ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#------------------------------------------------------------------------------
# Use importance distribution
#------------------------------------------------------------------------------



#------------------------------------------------------------------------------
# Gather sensor values
#------------------------------------------------------------------------------

# IEEE 802.15.4a signals are received from two transmitters 
# it is assumed that multiplexing is done
# the sensor frequency must be at least twice that of the unified frequency
IEEE_0 = 0 # pin readout from pin A
IEEE_1 = 0 # pin readout from pin A at an offest of < 0.5 * sensor_frequency

# Encoder values are received for the three omni-directional wheels
encoder_0 = 0 # pin readout from pin B
encoder_1 = 0 # pin readout from pin C
encoder_2 = 0 # pin readout from pin D

# Position is taken from IEEE 802.15.4a input
position_x, position_y = get_position_IEEE(IEEE_0, IEEE_1)

# Velocity is taken from encoder input (unreliable position because of drift)
x_previous, y_previous = x_present, y_present
x_present, y_present = get_position_encoder(encoder_0, encoder_1, encoder_2)
velocity_x = get_velocity(x_previous, x_present)
velocity_y = get_velocity(y_previous, y_present)


#------------------------------------------------------------------------------
# Update weights
#------------------------------------------------------------------------------



#------------------------------------------------------------------------------
# Normalize weights
#------------------------------------------------------------------------------



#------------------------------------------------------------------------------
# Output sensor-fused value
#------------------------------------------------------------------------------



#------------------------------------------------------------------------------
# Conduct resampling
#------------------------------------------------------------------------------


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------    Terminate Program    ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Release pigpio resources
pi.stop()