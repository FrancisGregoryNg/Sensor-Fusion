import numpy as np
import csv
import pigpio
import time

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------   Function Definitions  ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

def get_state(IEEE_0, IEEE_1, encoder_0, encoder_1):
    # Get the position using the IEEE 802.15.4a values
    
    # Use the empirically-determined factor for the inverse-square law
    # signal_strength = factor * (1 / distance ** 2)
    # Transmitter 0 is located at (0, 0) or (0, 0)
    # Transmitter 1 is located at (max_x, 0) or (room_width, 0)
    distance_0 = np.sqrt(factor_0 / IEEE_0)  
    distance_1 = np.sqrt(factor_1 / IEEE_1)
    
    # According to the SSS theorem, the angles of the triangle can be computed
    # Use the law of cosines: c ** 2 = a ** 2 + b ** 2 - 2 * a * b * cos(C)
    # Let angle_0 and angle_1 be the angles with respect to the horizontal
    # These are oriented so as to range from 0 to 90 degrees
    angle_0 = np.arccos((room_length ** 2 + distance_0 ** 2 - distance_1 ** 2)
                        / (2 * room_length * distance_0))
    angle_1 = np.arccos((room_length ** 2 + distance_1 ** 2 - distance_0 ** 2)
                        / (2 * room_length * distance_1))
    
    # Theoretically, only angles from one transmitter are needed
    # However, the calculation is not so heavy so the average may be taken
    position_x = ((distance_0 * np.cos(angle_0))
                  + (room_width - distance_1 * np.cos(angle_1))) / 2          
    position_y = ((distance_0 * np.sin(angle_0))
                  + (distance_1 * np.sin(angle_1)))/2

    # Get the position using the two encoder values

    # For two pairs of omni-wheels, with one encoder for each pair
    # Each pair is coupled so they move together
    # One pair moves in the x-direction, the other in the y-direction
    velocity_x = (encoder_0 / T) * np.pi * diameter * 0.5 / 360
    velocity_y = (encoder_1 / T) * np.pi * diameter * 0.5 / 360
    return position_x, position_y, velocity_x, velocity_y

# Used for gettting the sensor measurements
def get_measurements():
    # Get IEEE at the start
    
    # Request signal strength from the first transmitter
    pi.wave_send_once(get_strength_0)
    
    # Wait until all data has been sent
    while pi.wave_tx_busy():
          pass
    
    # Read signal strength data from the first transmitter
    (temp_data_0, IEEE_0_start) = pi.bb_serial_read(IEEE_0_RX)
    
    # Request signal strength from the second transmitter
    pi.wave_send_once(get_strength_1)
    
    # Wait until all data has been sent
    while pi.wave_tx_busy():
          pass
    
    # Read signal strength data from the second transmitter
    (temp_data_1, IEEE_1_start) = pi.bb_serial_read(IEEE_1_RX)
    
    # Convert bytearray to integer 
    IEEE_0_start = int.from_bytes(IEEE_0_start)  
    IEEE_1_start = int.from_bytes(IEEE_1_start)  
    
    # Set time for counter encoder rotations to T seconds (1 second)
    timeout = time.time() + T
    
    # Initialize the encoder counter
    encoder_0 = 0
    encoder_1 = 0
    
    # Take the initial measurements by getting the index in the sequence list
    # For encoder_0: (board 29, BCM 5) and (board 31, BCM 6)
    # For encoder_1: (board 32, BCM 12) and (board 33, BCM 13)
    current_0 = sequence.index((pi.read(5), pi.read(6))) 
    current_1 = sequence.index((pi.read(12), pi.read(13))) 
    
    # Loop until timeout occurs
    # Measurement is assumed to be quick enough to record increments of 1 or -1 
    while time.time() < timeout:
        
        # Consider previous measurements
        previous_0 = current_0
        previous_1 = current_1
        
        # Update the current measurements
        current_0 = sequence.index((pi.read(5), pi.read(6))) 
        current_1 = sequence.index((pi.read(12), pi.read(13))) 

        # Update the current position
        (encoder_0_A, encoder_0_B) = sequence(current_0)
        (encoder_1_A, encoder_1_B) = sequence(current_1)
        
        # Increment the encoder value by difference of the two positions
        increment_0 = (current_0 - previous_0)
        increment_1 = (current_1 - previous_1)
        
        # Value is wrong by a factor of -1/3 upon passing a sequence endpoint
        if increment_0 != (1 or -1):
            increment_0 == increment_0 * (-1/3)
        if increment_1 != (1 or -1):
            increment_1 == increment_1 * (-1/3)
            
        # Update the encoder counter
        encoder_0 += increment_0
        encoder_1 += increment_1
    
    # Get IEEE again at the end
    
    # Request signal strength from the first transmitter
    pi.wave_send_once(get_strength_0)
    
    # Wait until all data has been sent
    while pi.wave_tx_busy():
          pass
    
    # Read signal strength data from the first transmitter
    (temp_data_0, IEEE_0_end) = pi.bb_serial_read(IEEE_0_RX)
    
    # Request signal strength from the second transmitter
    pi.wave_send_once(get_strength_1)
    
    # Wait until all data has been sent
    while pi.wave_tx_busy():
          pass
    
    # Read signal strength data from the second transmitter
    (temp_data_1, IEEE_1_end) = pi.bb_serial_read(IEEE_1_RX)
    
    # Convert bytearray to integer 
    IEEE_0_end = int.from_bytes(IEEE_0_end)  
    IEEE_1_end = int.from_bytes(IEEE_1_end)  
    
    IEEE_0 = (IEEE_0_start + IEEE_0_end) / 2
    IEEE_1 = (IEEE_1_start + IEEE_1_end) / 2
    
    # Return values
    return IEEE_0, IEEE_1, encoder_0, encoder_1

# Used for predicting measurement values based on state values of a particle
def predict_measurements(position_x, position_y, velocity_x, velocity_y): 
        
    # Given the positions, the distances to the transmitters can be calculated
    # Transmitter 0 is located at (0, 0) or (0, 0)
    # Transmitter 1 is located at (max_x, 0) or (room_width, 0)
    distance_squared_0 = position_x ** 2 + position_y ** 2
    distance_squared_1 = (position_x - room_width) ** 2 + position_y ** 2
    
    # Use the empirically-determined factor for the inverse-square law
    # signal_strength = factor * (1 / distance ** 2)
    IEEE_0 = factor_0 / distance_squared_0
    IEEE_1 = factor_1 / distance_squared_1
    
    # Transpose the equation for getting the velocity with encoder values
    # velocity_x = (encoder_0 / T) * np.pi * diameter * 0.5 / 360
    # velocity_y = (encoder_1 / T) * np.pi * diameter * 0.5 / 360
    encoder_0 = (velocity_x * T) / (np.pi * diameter * 0.5 / 360)
    encoder_1 = (velocity_y * T) / (np.pi * diameter * 0.5 / 360)

    return IEEE_0, IEEE_1, encoder_0, encoder_1

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------      Initialization     ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#------------------------------------------------------------------------------
# Configure the pin connections
#------------------------------------------------------------------------------

# Use pigpio daemon to control general-purpose input/outputs more freely
pi = pigpio.pi()

# Set the pins for the IEEE 802.15.4a
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

# Set sensor pin inputs for the encoders

# Set pin A of encoder_0 (board 29, BCM 5) as input
pi.set_mode( 24, pigpio.INPUT)
# Set pin B of encoder_0 (board 31, BCM 6) as input
pi.set_mode( 25, pigpio.INPUT)

# Set pin A of encoder_1 (board 32, BCM 12) as input
pi.set_mode( 8, pigpio.INPUT)
# Set pin B of encoder_1 (board 33, BCM 13) as input
pi.set_mode( 7, pigpio.INPUT)

# Define the sequence of encoder values (cycles every 2 degrees of rotation)
sequence = ((0,0), (0,1), (1,1), (1,0))

#------------------------------------------------------------------------------
# Set up database
#------------------------------------------------------------------------------

# Initialize the signal mapping matrix and the corresponding location matrix
signal = np.zeros((1,2), dtype = float)
location = np.zeros((1,2), dtype = float)
    
# Copy data from the .csv files onto the corresponding matrices
with open('IEEE_signal_database.csv') as database:
    read_database = csv.DictReader(database)
    for row in read_database:
        signal = np.append(signal, 
                               [[float(row['IEEE_0']), float(row['IEEE_1'])]], 
                               axis = 0)
       
with open('IEEE_matching_locations.csv') as matching_locations:
    read_locations = csv.DictReader(matching_locations)
    for row in read_locations:
        location = np.append(location, 
                              [[float(row['x']), float(row['y'])]], 
                              axis = 0)
        
# Delete the initial zero-value rows of the matrices
signal = np.delete(signal, (0), axis = 0)
location = np.delete(location, (0), axis = 0)

#------------------------------------------------------------------------------
# Set values
#------------------------------------------------------------------------------

# number of particles is set heuristically
number_of_particles = 1000

# the room dimensions (width, length) correspond to (x, y)
# the origin starts at a selected corner of the room
# the measurement unit is in millimeters (mm)
room_width = 2000
room_length = 2000

# the maximum speed limit of the AGV is estimated (the lower the better)
# the measurement unit is in millimeters per second (mm/s)
# figure out how to incorporate this in noise calculations
max_speed = 1000   

# the diameter of each omni-wheel is in millimeters (mm)
diameter = 10

# time in seconds (s) wherein velocity is measured
T = 1

# covariance of process noise and measurement noise (guess: between 0 and 10)
covariance_process = 10 * np.random.random()
covariance_measurement = 10 * np.random.random()

#------------------------------------------------------------------------------
# Calculate proportionality factors for signal strength
#------------------------------------------------------------------------------
# Calculate the factor for the inverse-square law
# signal_strength = factor * (1 / distance ** 2)
# Transmitter 0 is located at (min_x, max_y) or (0, room_length)
# Transmitter 1 is located at (max_x, max_y) or (room_width, room_length)

# Create an array of ones for use in computations
number_of_samples = signal.shape[0]
ones = np.ones((number_of_samples, 1), dtype = float)

# Calculate the distances from the transmitters
distance_0 = np.sqrt(np.add(np.power(np.subtract(location[:][0], 
                                                 (room_width * ones)), 2), 
                            np.power(np.subtract(location[:][1], 
                                                 (room_length * ones)), 2)))
distance_1 = np.sqrt(np.add(np.power(np.subtract(location[:][0], 
                                                 (room_width * ones)), 2), 
                            np.power(np.subtract(location[:][1], 
                                                 (room_length * ones)), 2)))

# Calculate the individual proportionality factors for each data point
factor_0_individual = np.multiply(signal[:][0], np.power(distance_0, 2))
factor_1_individual = np.multiply(signal[:][1], np.power(distance_1, 2))

# Set the central tendency as the general proportionality factor 
factor_0 = np.median(factor_0_individual)
factor_1 = np.median(factor_1_individual)

#------------------------------------------------------------------------------
# Draw samples from a uniform distribution (initial distribution)
#------------------------------------------------------------------------------

# The basis for using uniform distribution is page 47 of XR-EE-SB 2012:015

# Initialize the state matrix with zeros
state_matrix = np.zeros((number_of_particles, 4), dtype = float)

# Fill state date for each particle
for particle in range(number_of_particles):
    
    # Use the constant velocity dynamic model
    # The state matrix is composed of the state vectors of all particles
    for state_vector_entry in range(4):
        
        # The initial distribution is 
        
        # state_vector_0 = x-position from IEEE 802.15.4a input
        state_matrix[particle][0] = np.random.uniform(0, room_width)
        
        # state_vector_1 = y-position from IEEE 802.15.4a input
        state_matrix[particle][1] = np.random.uniform(0, room_length)
        
        # state_vector_2 = x-velocity from encoder input
        state_matrix[particle][2] = np.random.uniform(-max_speed, max_speed)
        
         # state_vector_3 = y-velocity from encoder input
        state_matrix[particle][3] = np.random.uniform(-max_speed, max_speed)

#------------------------------------------------------------------------------
# Initialize the weights
#------------------------------------------------------------------------------

# Initialize the weights with zeros
weight = np.zeros((number_of_particles, 1), dtype = float)

# Initialize the temporary weights due to IEEE 802.15.4a and encoder values
weight_IEEE = np.zeros((number_of_particles, 1), dtype = float)
weight_encoder = np.zeros((number_of_particles, 1), dtype = float)

# Initialize the temporary arrays for resampling
resample = np.zeros((number_of_particles, 1), dtype = float)
weight_resample = np.zeros((number_of_particles, 1), dtype = float)
state_matrix_resample = np.zeros((number_of_particles, 4), dtype = float)

# The weights for the initial particles are equal
for particle in range(number_of_particles):
    weight[particle] = 1 / number_of_particles
    
#------------------------------------------------------------------------------
# Prepare measurements for main loop
#------------------------------------------------------------------------------

# Get IEEE 802.15.4a and encoder values
IEEE_0, IEEE_1, encoder_0, encoder_1 = get_measurements()

# Prepare the matrix for storing predicted measurements
# IEEE_0, IEEE_1, encoder_0, encoder_1
predicted = np.zeros((number_of_particles, 4), dtype = float)

# Prepare the matrix for storing white Gaussian noise values
noise_process = np.zeros((number_of_particles, 4), dtype = float)
noise_measurement = np.zeros((number_of_particles, 1), dtype = float)

# Prepare the factors for updating the state matrix
state_factor = np.array([[1, 1, T, T],
                         [1, 1, T, T],
                         [0, 0, 1, 1],
                         [0, 0, 1, 1]])
noise_factor = np.array([[0.5 * T **2, 0.5 * T **2],
                         [0.5 * T **2, 0.5 * T **2],
                         [T, T],
                         [T, T]])

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------  Main Loop (Iterations) ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Repeat the loop indefinitely
while True:

#------------------------------------------------------------------------------
# Measurement prediction
#------------------------------------------------------------------------------
    
    # Randomize process noise (mean is zero (white noise))
    mean = 0
    standard_deviation_process = np.sqrt(covariance_measurement)
    noise_process = np.random.normal(mean, standard_deviation_process, 
                                     (number_of_particles, 4))
    
    for particle in range(number_of_particles):
        # IEEE_0, IEEE_1, encoder_0, encoder_1
        predicted[particle] = predict_measurements(state_matrix[particle][0],
                                                   state_matrix[particle][1],
                                                   state_matrix[particle][2],
                                                   state_matrix[particle][3])
    
#------------------------------------------------------------------------------
# Actual measurement
#------------------------------------------------------------------------------

    # Get IEEE 802.15.4a and encoder values
    IEEE_0, IEEE_1, encoder_0, encoder_1 = get_measurements()
    
#------------------------------------------------------------------------------
# Modify the weights
#------------------------------------------------------------------------------

    # Create an array of ones for use in computations
    ones = np.ones((number_of_particles, 1), dtype = float)

    # Randomize measurement noise
    mean = 0
    standard_deviation_measurement = np.sqrt(covariance_measurement)
    noise_measurement = np.random.normal(mean, standard_deviation_measurement,
                                         (number_of_particles, 1))  
    
    # Update the weights based on IEEE 802.15.4a values
    # Take the average of the errors from the two sensors
    # Use the first two columns from the measurement prediction matrix
    IEEE = np.concatenate((IEEE_0 * ones, IEEE_1 * ones), axis = 1)
    difference = np.mean(np.subtract(IEEE, predicted[:][1:3]), axis = 1)                               
    weight_IEEE = np.multiply(np.multiply(weight, difference), 
                              noise_measurement)
        
    # Normalize IEEE 802.15.4a weights 
    weight_total = np.sum(weight_IEEE) * ones
    weight_IEEE = np.divide(weight_IEEE, weight_total)

    # Update the weights based on encoder values
    # Take the average of the errors from the two sensors
    # Use the last two columns from the measurement prediction matrix
    encoder = np.concatenate((encoder_0 * ones, encoder_1 * ones), axis = 1)
    difference = np.mean(np.subtract(encoder, predicted[:][3:]), axis = 1) 
    weight_encoder = np.multiply(np.multiply(weight, difference),
                                 noise_measurement)
         
    # Normalize encoder weights
    weight_total = np.sum(weight_encoder) * ones
    weight_encoder = np.divide(weight_encoder, weight_total)
        
    # Get the mean IEEE 802.15.4a weights and encoder weights
    # This is needed because the two sensor values cannot be directly averaged
    # The normalized weights, however, can be averaged quite easily
    weight = np.mean(np.concatenate((weight_IEEE, weight_encoder), axis = 1),
                     axis = 1)
        
#------------------------------------------------------------------------------
# Output sensor-fused value
#------------------------------------------------------------------------------

    # Get the position and velocities of the particles
    for particle in range(number_of_particles): 
        # from IEEE 802.15.4a and encoder input
        state_matrix[particle] = get_state(IEEE[particle], encoder[particle])

    # The estimated position values are the main output
    # These values are used for the next iteration
    # Get the summation of the element-wise product of values and weights
    estimated_position_x = np.sum(np.multiply(state_matrix[:][0], weight))
    estimated_position_y = np.sum(np.multiply(state_matrix[:][1], weight))
    estimated_velocity_x = np.sum(np.multiply(state_matrix[:][2], weight))
    estimated_velocity_y = np.sum(np.multiply(state_matrix[:][3], weight))
    
#------------------------------------------------------------------------------
# Conduct resampling
#------------------------------------------------------------------------------

    # Resample if the number of effective particles is below a threshold
    effective_number_of_particles = 1 / np.sum(np.power(weight, 2))
    
    if effective_number_of_particles < 2/3 * number_of_particles:
        # The systematic within residual resampling method is used
        # The basis for this is page 21 of by Murray Pollock's 
        # “Introduction to Particle Filtering” Discussion - Notes
        
        # Get the integer part and the residual part of the scaled weights
        (number_of_copies, residual) = np.divmod(number_of_particles * weight)
        
        # Get the needed values for the resampling method
        residual = residual / np.sum(residual)
        cumulative_sum = np.cumsum(residual)
        divisions = number_of_particles - np.sum(number_of_copies)
        positions = (np.linspace(0, 1, divisions, endpoint = False) +
                     (np.random.random() / divisions))
        
        # Make sure that the sequence ends with 1 (not more or less)
        cumulative_sum[-1] = 1
        
        # Evaluate the particles based on the determined values
        # Conducted residual and systematic strategies per particle
        # This makes it easier to count the number of identical samples
        # It may be possible that doing each strategy as a block can be better
        selected = 0
        current_division = 0
        for particle in range(number_of_particles): 
            for copy in range(number_of_copies[particle]):
                resample[selected] = particle
                selected += 1
            if positions[current_division] <= cumulative_sum[particle]:
                resample[selected] = particle
                selected += 1
                current_division +=1 
                
        # Update the state matrix
        # Take the elements according to the chosen resampling indices
        state_matrix_resample = np.take(state_matrix, resample)
        state_matrix = state_matrix_resample
        
        # Update the weights
        weight_resample = np.take(weight, resample)
        weight = weight_resample

#------------------------------------------------------------------------------
# Update the particles
#------------------------------------------------------------------------------

    # Update the state of the particle based on noise and previous state
    # The previous state was used for measurement prediction
    # The basis for this is page 47 of 978-1580536318/158053631X
    state_update = np.matmul(state_factor, state_matrix[particle][0]) 
    noise_update = np.matmul(noise_factor, noise_process[particle])
    state_matrix[particle] = np.add(state_update, noise_update)
        
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------    Terminate Program    ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Close pigpio
pi.wave_delete(0)
pi.wave_delete(1)
pi.bb_serial_read_close(IEEE_0_RX)
pi.bb_serial_read_close(IEEE_1_RX)
pi.stop()