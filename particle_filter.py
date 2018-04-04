import numpy as np
from numpy.random import uniform
import csv
'''import pigpio'''
'''import time'''

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------   Function Definitions  ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Used for getting the position using the IEEE 802.15.4a values
def get_position(IEEE_0_start, IEEE_0_end, IEEE_1_start, IEEE_1_end):
    # Use the empirically-determined factor for the inverse-square law
    # signal_strength = factor * (1 / distance ^ 2)
    # Transmitter 0 is located at (min_x, max_y) or (0, room_length)
    # Transmitter 1 is located at (max_x, max_y) or (room_width, room_length)
    distance_0_start = np.sqrt(factor_0 / IEEE_0_start)  
    distance_1_start = np.sqrt(factor_1 / IEEE_1_start)
    distance_0_end = np.sqrt(factor_0 / IEEE_0_end)
    distance_1_end = np.sqrt(factor_1 / IEEE_1_end)
    # According to the SSS theorem, the angles of the triangle can be computed
    # Use the law of cosines: c ** 2 = a ** 2 + b ** 2 - 2 * a * b * cos(C)
    # Let angle_0 and angle_1 be the angles with respect to the horizontal
    # These are oriented so as to range from 0 to 90 degrees
    angle_0_start = np.arccos((room_length ** 2
                               + distance_0_start ** 2
                               - distance_1_start ** 2)
                              / 2 * room_length * distance_0_start)
    angle_1_start = np.arccos((room_length ** 2
                               + distance_1_start ** 2
                               - distance_0_start ** 2)
                              / 2 * room_length * distance_1_start)                       
    angle_0_end = np.arccos((room_length ** 2
                             + distance_0_end ** 2
                             - distance_1_end ** 2)
                            / 2 * room_length * distance_0_end)
    angle_1_end = np.arccos((room_length ** 2
                             + distance_1_end ** 2
                             - distance_0_end ** 2)
                            / 2 * room_length * distance_1_end)
    # Theoretically, only angles from one transmitter are needed
    # However, the calculation is not so heavy so the average may be taken
    x_start = (distance_0_start * np.cos(angle_0_start)
               + (room_width - distance_1_start * np.cos(angle_1_start))) / 2          
    y_start = ((room_length - distance_0_start * np.sin(angle_0_start))
               + (room_length - distance_1_start * np.sin(angle_1_start)))/2
    x_end = (distance_0_end * np.cos(angle_0_end)
             + (room_width - distance_1_end * np.cos(angle_1_end))) / 2
    y_end = ((room_length - distance_0_end * np.sin(angle_0_end))
             + (room_length - distance_1_end * np.sin(angle_1_end)))/2
    # Get the location between the start and end points
    position_x = (x_start + x_end) / 2
    position_y = (y_start + y_end) / 2
    return position_x, position_y
-------------------------------------------------------------------------------

# Used for getting the position using the three encoder values
def get_velocity(encoder_0, encoder_1, encoder_2):
    # For a delta configuration of the omni-wheels /_\
    # (\) Right is encoder_0, 
    # (/) Left is encoder_1, 
    # (_) Bottom is encoder_2
    # Treat direction along clockwise rotation as positive
    # Get the mean measurement along each axis
    # velocity_x    = 0.5 * encoder_0_for_x 
    #               = -0.5 * encoder_1_for_x
    #               = encoder_2
    # velocity_y    = 0.57735 * encoder_0_for_y
    #               = 0.57735 * encoder_1_for_y
    velocity_x = encoder_2
    velocity_y = 0.57735 * (((encoder_0 + encoder_1) / 2) - 2 * velocity_x)
    # The encoder reflects a 0.5-degree rotation per value change
    # Convert encoder rate into velocity in millimeters per second (mm/s)
    velocity_x = (velocity_x / T) * np.pi * diameter * 0.5 / 360
    velocity_y = (velocity_y / T) * np.pi * diameter * 0.5 / 360
    return velocity_x, velocity_y

'''
# Used for gettting the sensor measurements
def get_measurements():
    # Open pins for bit bang reading of serial data
    pi.bb_serial_read_open(14, 115200)
    pi.bb_serial_read_open(15, 115200) 
    
    # IEEE 802.15.4a signals are taken before the encoder logging time
    # Base signal strength on the number of bytes read, ignore bytearray
    IEEE_0_start, temp_data_0) = pi.bb_serial_read()
    (IEEE_1_start, temp_data_1) = pi.bb_serial_read() 
    
    # Close pins for bit bang reading of serial data
    pi.bb_serial_read_close(14)
    pi.bb_serial_read_close(15)
    
    # Set time for counter encoder rotations to T seconds (1 second)
    timeout = time.time() + T
    
    # Initialize the encoder counter
    encoder_0 = 0
    encoder_1 = 0
    encoder_2 = 0
    
    # Take the initial measurements
    current_0 = sequence.index((pi.read(5), pi.read(6))) 
    current_1 = sequence.index((pi.read(12), pi.read(13))) 
    current_2 = sequence.index((pi.read(19), pi.read(26)))
    
    # Loop until timeout occurs
    # Measurement is assumed to be quick enough to record increments of 1 or -1 
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
        
        # Value is wrong by a factor of -1/3 upon passing a sequence endpoint
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
    
    # Return values
    return (IEEE_0_start, IEEE_0_end,
            IEEE_1_start, IEEE_1_end,
            encoder_0, encoder_1, encoder_2)
'''
# Used for predicting IEEE 802.15.4a values based on state values of a particle
def predict_IEEE(position_x, position_y, velocity_x, velocity_y):
    # Get the scale of acceptable offset for the x position      
    grid_resolution_x = room_width / (np.size(location, 0) -1)    
    scale_x = np.round(velocity_x / grid_resolution_x)
    if not(0 <= position_x + scale_x * grid_resolution_x <= room_width and
           0 <= position_x - scale_x * grid_resolution_x <= room_width):
        if (0 <= position_x + grid_resolution_x <= room_width and
            0 <= position_x - grid_resolution_x <= room_width):
            scale_x = 1
        else:
            scale_x = 0   
            
    # Get the scale of acceptable offset for the y position   
    grid_resolution_y = room_length / (np.size(location, 1) -1)    
    scale_y = np.round(velocity_y / grid_resolution_y)
    if not(0 <= position_y + scale_y * grid_resolution_y <= room_length and
           0 <= position_y - scale_y * grid_resolution_y <= room_length):
        if (0 <= position_y + grid_resolution_y <= room_length and
            0 <= position_y - grid_resolution_y <= room_length):
            scale_y = 1
        else:
            scale_y = 0    
    
    # Get the positions with offest corresponding to start and end
    position_x_start = position_x - scale_x * grid_resolution_x   
    position_y_start = position_y - scale_y * grid_resolution_y
    position_x_end = position_x + scale_x * grid_resolution_x   
    position_y_end = position_y + scale_y * grid_resolution_y 
     
    # Get the corresponding index for the positions
    index_start = bestfit(location, position_x_start, position_y_start)
    index_end = bestfit(location, position_x_end, position_y_end)
    
    # Get the corresponding IEEE 802.15.4a signals
    IEEE_0_start, IEEE_1_start = signal_map[index_start]
    IEEE_0_end, IEEE_1_end = signal_map[index_end]
    return IEEE_0_start, IEEE_0_end, IEEE_1_start, IEEE_1_end

# Used for predicting encoder values based on state values of a particle
def predict_encoder(velocity_x, velocity_y):
    # velocity_x    = 0.5 * encoder_0_for_x 
    #               = -0.5 * encoder_1_for_x
    #               = encoder_2
    # velocity_y    = 0.57735 * encoder_0_for_y
    #               = 0.57735 * encoder_1_for_y
    velocity_x = (velocity_x * T) / (np.pi * diameter * 0.5 / 360)
    velocity_y = (velocity_y * T) / (np.pi * diameter * 0.5 / 360)
    encoder_0 = (velocity_y + 2 * velocity_x) / 0.57735
    encoder_1 = (velocity_y + 2 * velocity_x) / 0.57735
    encoder_2 = velocity_x
    return encoder_0, encoder_1, encoder_2

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------      Initialization     ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#------------------------------------------------------------------------------
# Configure the pin connections
#------------------------------------------------------------------------------
'''
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
pi.set_mode( 26, pigpio.INPUT)'''

# Define the sequence of encoder values (cycles every 2 degrees of rotation)
sequence = ((0,0), (0,1), (1,1), (1,0))

#------------------------------------------------------------------------------
# Set up database
#------------------------------------------------------------------------------

# Initialize the signal mapping matrix and the corresponding location matrix
signal_map = np.zeros((1,2), dtype = float)
location = np.zeros((1,2), dtype = float)
    
# Copy data from the .csv files onto the corresponding matrices
with open('IEEE_signal_database.csv') as database:
    read_database = csv.DictReader(database)
    for row in read_database:
        signal_map = np.append(signal_map, 
                               [[float(row['IEEE_0']), float(row['IEEE_1'])]], 
                               axis = 0)
       
with open('IEEE_matching_locations.csv') as matching_locations:
    read_locations = csv.DictReader(matching_locations)
    for row in read_locations:
        location = np.append(location, 
                              [[float(row['x']), float(row['y'])]], 
                              axis = 0)
        
# Delete the initial zero-value rows of the matrices
signal_map = np.delete(signal_map, (0), axis = 0)
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
''' figure out how to incorporate this in noise calculations '''
max_speed = 1000   

# the diameter of each omni-wheel is in millimeters (mm)
diameter = 10

# time in seconds (s) wherein velocity is measured
T = 1

# covariance of process noise and measurement noise (guess: between 0 and 10)
covariance_process = 10 * np.random.random()
covariance_measurement = 10 * np.random.random()

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
        state_matrix[particle][0] = uniform(low = 0, high = room_width)
        
        # state_vector_1 = y-position from IEEE 802.15.4a input
        state_matrix[particle][1] = uniform(low = 0, high = room_length)
        
        # state_vector_2 = x-velocity from encoder input
        state_matrix[particle][2] = uniform(low = -max_speed, high = max_speed)
        
         # state_vector_3 = y-velocity from encoder input
        state_matrix[particle][3] = uniform(low = -max_speed, high = max_speed)

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
'''(IEEE_0_start, IEEE_0_end,
 IEEE_1_start, IEEE_1_end,
 encoder_0, encoder_1, encoder_2) = get_measurements()'''

# Prepare the matrix for storing predicted measurements
IEEE = np.zeros((number_of_particles, 2), dtype = float)
encoder = np.zeros((number_of_particles, 3), dtype = float)

# Prepare the matrix for storing white Gaussian noise values
noise_process = np.zeros((number_of_particles, 6), dtype = float)
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

#------------------------------------------------------------------------------
# Conduct tests
#------------------------------------------------------------------------------

# IEEE 802.15.4a testing
IEEE_0_start = np.random.uniform(0, 255)
IEEE_0_end = np.random.uniform(0, 255)
IEEE_1_start = np.random.uniform(0, 255)
IEEE_1_end = np.random.uniform(0, 255)
print("IEEE_0_start:", IEEE_0_start)
print("IEEE_0_end:", IEEE_0_end)
print("IEEE_1_start:", IEEE_1_start)
print("IEEE_1_end:", IEEE_1_end)
position = get_position(IEEE_0_start, IEEE_0_end, 
                        IEEE_1_start, IEEE_1_end)
print("position: ", str(position))
i = bestfit(signal_map, IEEE_0_start, IEEE_1_start)
j = bestfit(signal_map, IEEE_0_end, IEEE_1_end)
print("location[i]: ", location[i])
print("location[j]: ", location[j])

# encoder testing
encoder_0 = np.random.uniform(0, 500)
encoder_1 = np.random.uniform(0, 500)
encoder_2 = np.random.uniform(0, 500)
print("encoder_0:", encoder_0)
print("encoder_1:", encoder_1)
print("encoder_2:", encoder_2)
velocity = get_velocity(encoder_0, encoder_1, encoder_2)
print("velocity: ", str(velocity))

# measurement prediction testing
position_x = np.random.uniform(0, 2000)
position_y = np.random.uniform(0, 2000)
velocity_x = np.random.uniform(0, 50)
velocity_y = np.random.uniform(0, 50)
print("position_x:", position_x)
print("position_y:", position_y)
print("velocity_x:", velocity_x)
print("velocity_y:", velocity_y)
IEEE = predict_IEEE(position_x, position_y, velocity_x, velocity_y)
encoder = predict_encoder(velocity_x, velocity_y)
print("IEEE: ", str(IEEE))
print("encoder", str(encoder))    
position_x_resolve, position_y_resolve = get_position(IEEE[0], IEEE[1], 
                                                      IEEE[2], IEEE[3])
velocity_x_resolve, velocity_y_resolve = get_velocity(encoder[0], 
                                                      encoder[1], 
                                                      encoder[2])
print("position_x_resolve: ", str(position_x_resolve))
print("position_y_resolve: ", str(position_y_resolve))
print("velocity_x_resolve", str(velocity_x_resolve))    
print("velocity_y_resolve", str(velocity_y_resolve)) 

    
'''
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------  Main Loop (Iterations) ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Repeat the loop indefinitely
while True:

#------------------------------------------------------------------------------
# Measurement prediction
#------------------------------------------------------------------------------
    
    # Randomize process noise (mean is zero (white noise))
    noise_process = np.random.normal(0, np.sqrt(covariance_process), 
                                     (number_of_particles, 4))
    
    for particle in range(number_of_particles):
        # IEEE_0_start, IEEE_0_end, IEEE_1_start, IEEE_1_end
        IEEE[particle] = predict_IEEE(state_matrix[particle][0],
                                      state_matrix[particle][1],
                                      state_matrix[particle][2],
                                      state_matrix[particle][3])
        
        # encoder_0, encoder_1, encoder_2
        encoder[particle] = predict_encoder(state_matrix[particle][2], 
                                            state_matrix[particle][3]) 
        
        # Update the state of the particle based on noise and previous state
        # The previous state was used for measurement prediction
        # The basis for this is page 47 of 978-1580536318/158053631X
        state_update = np.matmul(state_factor, state_matrix[particle][0]) 
        noise_update = np.matmul(noise_factor, noise_process[particle])
        state_matrix[particle] = np.add(state_update, noise_update)
    
#------------------------------------------------------------------------------
# Actual measurement
#------------------------------------------------------------------------------

    # Get IEEE 802.15.4a and encoder values
    (IEEE_0_start, IEEE_0_end,
     IEEE_1_start, IEEE_1_end,
     encoder_0, encoder_1, encoder_2) = get_measurements()
    
#------------------------------------------------------------------------------
# Modify the weights
#------------------------------------------------------------------------------

    # Randomize measurement noise
    noise_measurement = np.random.normal(0, np.sqrt(covariance_measurement),
                                         (number_of_particles, 1))  
    
    # Update the weights based on IEEE 802.15.4a values
    for particle in range(number_of_particles):
        difference = np.mean(np.subtract(np.array([IEEE_0_start, IEEE_0_end,
                                                   IEEE_1_start, IEEE_1_end]),
                                         IEEE[particle]))                               
        weight_IEEE[particle] = (weight[particle] * difference *
                                 noise_measurement[particle])
        
    # Normalize IEEE 802.15.4a weights 
    weight_total = np.sum(weight_IEEE)
    for particle in range(number_of_particles):
        weight_IEEE[particle] = weight_IEEE[particle] / weight_total

    # Update the weights based on encoder values
    for particle in range(number_of_particles):
        difference = np.mean(np.subtract(np.array([encoder_0, 
                                                   encoder_1, 
                                                   encoder_2]),
                                         encoder[particle]))                        
        weight_encoder[particle] = (weight[particle] * difference * 
                                    noise_measurement[particle])
         
    # Normalize encoder weights
    weight_total = np.sum(weight_encoder)
    for particle in range(number_of_particles):
        weight_encoder[particle] = weight_encoder[particle] / weight_total
        
    # Get the mean IEEE 802.15.4a weights and encoder weights
    # This is needed because the two sensor values cannot be directly averaged
    # The normalized weights, however, can be averaged quite easily
    ''' '''
    This will supposedly create a 3D array from the two [n, 1] arrays.
    Selecting axis = 0 in this case will make the average act between the
    two arrays.
    However, check if this instead creates a 2D array with a dimension of 
    [n, 2]. This will instead require the use of axis = 1.
    ''' '''
    weight = np.mean(np.array([weight_IEEE, weight_encoder]), axis = 0)
        
#------------------------------------------------------------------------------
# Output sensor-fused value
#------------------------------------------------------------------------------

    # Get the position and velocities of the particles
    for particle in range(number_of_particles): 
        # from IEEE 802.15.4a input
        state_matrix[particle][0:1] = get_position(IEEE[particle])
        
        # from encoder input
        state_matrix[particle][2:3] = get_velocity(encoder[particle])

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
        state_matrix_resample = np.take(state_matrix, resample)
        state_matrix = state_matrix_resample
        
        # Update the weights
        weight_resample = np.take(weight, resample)
        weight = weight_resample

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------    Terminate Program    ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Release pigpio resources
pi.stop()
'''