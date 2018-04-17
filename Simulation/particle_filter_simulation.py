import numpy as np
import time
import matplotlib as plt
import agv_library as agv

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------   Function Definitions  ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def get_IEEE_plot_data(IEEE_0, IEEE_1):
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
                  + (distance_1 * np.sin(angle_1))) / 2
    return position_x, position_y

def get_encoder_plot_data(encoder_0, encoder_1, prior_x, prior_y):
    position_x = encoder_0 * np.pi * diameter * 0.5 / 360
    position_y = encoder_1 * np.pi * diameter * 0.5 / 360
    return position_x, position_y

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
                  + (distance_1 * np.sin(angle_1))) / 2

    # Get the position using the two encoder values

    # For two pairs of omni-wheels, with one encoder for each pair
    # Each pair is coupled so they move together
    # One pair moves in the x-direction, the other in the y-direction
    velocity_x = (encoder_0 / T) * np.pi * diameter * 0.5 / 360
    velocity_y = (encoder_1 / T) * np.pi * diameter * 0.5 / 360
    return position_x, position_y, velocity_x, velocity_y

# Used for gettting the sensor measurements
def get_measurements():
    roboman.updateModel()
    
    # Get IEEE at the start
    IEEE_0_start, IEEE_1_start = roboman.readIEEE()
    
    # Set time for counter encoder rotations to T seconds (1 second)
    timeout = time.time() + T
    
    # Initialize the encoder
    encoder_0_start, encoder_1_start = roboman.readEncoder()
    
    # Loop until timeout occurs
    # Measurement is assumed to be quick enough to record increments of 1 or -1 
    while time.time() < timeout:
        roboman.updateModel()
    
    encoder_0_end, encoder_1_end = roboman.readEncoder()
    
    # Get IEEE again at the end
    IEEE_0_end, IEEE_1_end = roboman.readIEEE()
    
    IEEE_0 = (IEEE_0_start + IEEE_0_end) / 2
    IEEE_1 = (IEEE_1_start + IEEE_1_end) / 2
    
    encoder_0 = encoder_0_end - encoder_0_start
    encoder_1 = encoder_1_end - encoder_1_start
    
    roboman.updateModel()
    # Return values
    return IEEE_0, IEEE_1, encoder_0, encoder_1

# Get the signal equivalent of IEEE because simulation gives the position
def get_IEEE(position_x, position_y):
    # Given the positions, the distances to the transmitters can be calculated
    # Transmitter 0 is located at (0, 0) or (0, 0)
    # Transmitter 1 is located at (max_x, 0) or (room_width, 0)
    distance_squared_0 = position_x ** 2 + position_y ** 2
    distance_squared_1 = (position_x - room_width) ** 2 + position_y ** 2
    
    # Use the empirically-determined factor for the inverse-square law
    # signal_strength = factor * (1 / distance ** 2)
    IEEE_0 = factor_0 / distance_squared_0
    IEEE_1 = factor_1 / distance_squared_1
    
    return IEEE_0, IEEE_1
    
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
roboman = agv.Vehicle(0.05, 1, 0.5, 0.01, 0, 0, 1, 10, 0, 0)

starting_x = 0
starting_y = 0

# Define the sequence of encoder values (cycles every 2 degrees of rotation)
sequence = ((0,0), (0,1), (1,1), (1,0))

actual_plot_data = np.zeros((1, 2), dtype = float)
IEEE_plot_data = np.zeros((1, 2), dtype = float)
encoder_plot_data = np.zeros((1, 2), dtype = float)
estimated_plot_data = np.zeros((1, 2), dtype = float)

encoder_position_x = starting_x
encoder_position_y = starting_y
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
diameter = 50

# time in seconds (s) wherein velocity is measured
T = 0.10

# covariance of process noise and measurement noise (guess: between 0 and 10)
covariance_process = 0.25 * np.random.random()
covariance_measurement = 0.25 * np.random.random()

# proportionality factors for signal strength
factor_0 = 5
factor_1 = 5

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
print("Start of main loop\n")
# Repeat the loop for a given amount of time
duration = 10
iteration = 0
start = time.time()
stop_loop = time.time() + duration
while time.time() < stop_loop:
    iteration += 1
    print("Iteration # " + str(iteration), end = "")
    print("; Time = " + "{:.2f}".format(time.time() - start) + "s")        
#------------------------------------------------------------------------------
# Measurement prediction
#------------------------------------------------------------------------------
    
    # Randomize process noise (mean is zero (white noise))
    mean = 0
    standard_deviation_process = np.sqrt(covariance_measurement)
    noise_process = np.random.normal(mean, standard_deviation_process, 
                                     (number_of_particles, 2))
    
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
    
    # Get actual position from simulation
    actual_position_x, actual_position_y = roboman.readActual()
    
    # Actual position for plot data
    new_actual = np.array([[actual_position_x, actual_position_y]])
    actual_plot_data = np.append(actual_plot_data, new_actual, axis = 0)
    
    # Get position if only purely IEEE 802.15.4a data is used for localization
    IEEE_position_x, IEEE_position_y = IEEE_0, IEEE_1
    
    # Get the actual measurements because simulation gave positions
    IEEE_0, IEEE_1 = get_IEEE(IEEE_0, IEEE_1)
    
    # Record IEEE position for plot data
    new_IEEE = np.array([[IEEE_position_x, IEEE_position_y]])
    IEEE_plot_data = np.append(IEEE_plot_data, new_IEEE, axis = 0)
    
    # Get position if only purely encoder data is used for localization
    (encoder_position_x,
     encoder_position_y) = get_encoder_plot_data(encoder_0, encoder_1, 
                                                 encoder_position_x,
                                                 encoder_position_y)
        
    # Record encoder position for plot data
    new_encoder = np.array([[encoder_position_x, encoder_position_y]])
    encoder_plot_data = np.append(encoder_plot_data, new_encoder, axis = 0)
    
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
    difference = np.mean(np.absolute(np.subtract(IEEE, predicted[:, [0,1]])), 
                         axis = 1)                               
    weight_IEEE = np.multiply(np.multiply(weight, difference), 
                              noise_measurement)
        
    # Normalize IEEE 802.15.4a weights 
    weight_total = np.sum(weight_IEEE) * ones
    weight_IEEE = np.divide(weight_IEEE, weight_total)

    # Update the weights based on encoder values
    # Take the average of the errors from the two sensors
    # Use the last two columns from the measurement prediction matrix
    encoder = np.concatenate((encoder_0 * ones, encoder_1 * ones), axis = 1)
    difference = np.mean(np.absolute(np.subtract(encoder, predicted[:,[2,3]])), 
                         axis = 1) 
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
    
    # Normalize weights
    weight_total = np.sum(weight) * ones
    weight = np.divide(weight, weight_total)
#------------------------------------------------------------------------------
# Output sensor-fused value
#------------------------------------------------------------------------------

    # Get the position and velocities of the particles
    for particle in range(number_of_particles): 
        # from IEEE 802.15.4a and encoder input
        state_matrix[particle] = get_state(IEEE[particle, 0],
                                           IEEE[particle, 1],
                                           encoder[particle, 0],
                                           encoder[particle, 1])

    # The estimated position values are the main output
    # These values are used for the next iteration
    # Get the summation of the element-wise product of values and weights
    estimated_position_x = np.sum(np.multiply(state_matrix[:, 0], weight))
    estimated_position_y = np.sum(np.multiply(state_matrix[:, 1], weight))
    estimated_velocity_x = np.sum(np.multiply(state_matrix[:, 2], weight))
    estimated_velocity_y = np.sum(np.multiply(state_matrix[:, 3], weight))
    
    print("\t\t\t\tEstimated x = " + str(estimated_position_x))
    print("\t\t\t\tEstimated y = " + str(estimated_position_y))
    #print("\tCheck = " + str(check))
    # Record estimated position for plot data
    new_estimate = np.array([[estimated_position_x, estimated_position_y]])
    estimated_plot_data = np.append(estimated_plot_data, new_estimate, 
                                    axis = 0)
    
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
        (number_of_copies, residual) = np.divmod(number_of_particles * weight, 
                                                 1)
        
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
            for copy in range(int(number_of_copies[particle])):
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
    state_update = np.matmul(state_factor, state_matrix[particle, :]) 
    noise_update = np.matmul(noise_factor, noise_process[particle, :])
    state_matrix[particle] = np.add([state_update], [noise_update])

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#------------------------ -  Consolidate Results   ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#------------------------------------------------------------------------------
# Convert numpy array to list
#------------------------------------------------------------------------------

actual_vertices = np.ndarray.tolist(actual_plot_data[:])
IEEE_vertices = np.ndarray.tolist(IEEE_plot_data[:])
encoder_vertices = np.ndarray.tolist(encoder_plot_data[:])
estimated_vertices = np.ndarray.tolist(estimated_plot_data[:])

#------------------------------------------------------------------------------
# Plot the different position data
#------------------------------------------------------------------------------
