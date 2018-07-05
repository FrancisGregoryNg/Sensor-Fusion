import numpy as np
import matplotlib.pyplot as plt
import agv_library as agv

def get_state(IEEE_0, IEEE_1, encoder_0, encoder_1):
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

    # For two pairs of omni-wheels, with one encoder for each pair
    # Each pair is coupled so they move together
    # One pair moves in the x-direction, the other in the y-direction
    velocity_x = (encoder_0 / T) * np.pi * diameter * 0.5 / 360
    velocity_y = (encoder_1 / T) * np.pi * diameter * 0.5 / 360
    
    # Return values
    return position_x, position_y, velocity_x, velocity_y

###############################################################################    
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------      Initialization     ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
###############################################################################
   
#------------------------------------------------------------------------------
# Define the parameters
#------------------------------------------------------------------------------
    
# number of particles is set heuristically
number_of_particles = 1000

# Non-particle filter
lookback_scale = 5
previous_position_x = 0
previous_position_y = 0
previous_velocity_x = 0
previous_velocity_y = 0
state_vector = np.zeros(4, dtype = float)
n_estimated_plot_data = np.zeros((1, 2), dtype = float)

# Define the covariance of process noise and measurement noise
std_dev_process = 0.0605935153135
std_dev_measurement = 9.68756654964

# Mean is zero (white noise) for both process and measurement noise 
mean = 0

# Define the time in seconds (s) wherein velocity is measured
T = 0.1

# Define the room dimensions (width, length) correspond to (x, y)
# The origin starts at a selected corner of the room
# The measurement unit is in meters (m)
room_width = 5
room_length = 5

# Define the diameter of each omni-wheel is in meters (m)
diameter = 0.024 * 2

# Define the proportionality factors for signal strength
factor_0 = 1
factor_1 = 1

#------------------------------------------------------------------------------
# Prepare empty/basic arrays and values
#------------------------------------------------------------------------------
    
# Set the starting state to zero
starting_position_x = 0
starting_position_y = 0
starting_velocity_x = 0
starting_velocity_y = 0

# Prepare starting position for encoder position update
encoder_position_x = starting_position_x
encoder_position_y = starting_position_y

# Create an array of ones for use in computations
ones = np.ones((number_of_particles, 1), dtype = float)

# Prepare the matrix for storing predicted measurements
# IEEE_0, IEEE_1, encoder_0, encoder_1
predicted = np.zeros((number_of_particles, 2), dtype = float)

   
# Prepare the matrix for storing white Gaussian noise values
noise_measurement = np.zeros((number_of_particles, 1), dtype = float)
noise_process = np.zeros((number_of_particles, 2), dtype = float)

# Prepare the arrays to store the position histories for plotting
actual_plot_data = np.zeros((1, 2), dtype = float)
IEEE_plot_data = np.zeros((1, 2), dtype = float)
encoder_plot_data = np.zeros((1, 2), dtype = float)
estimated_plot_data = np.zeros((1, 2), dtype = float)

# Initialize the weights as equal for all particles
weight = np.ones((number_of_particles, 1), dtype = float)
weight = weight / number_of_particles

# Initialize values for resampling
scaled_weight = np.zeros((number_of_particles, 1), dtype = float)
number_of_copies = np.zeros((number_of_particles, 1), dtype = float)
residual = np.zeros((number_of_particles, 1), dtype = float)
normalized_residual = np.zeros((number_of_particles, 1), dtype = float)
cumulative_sum = np.zeros((number_of_particles, 1), dtype = float)
resampling_indices = np.zeros((number_of_particles, 1), dtype = float) 

# Initialize the state matrix with zeros
# Use the constant velocity dynamic model
# Entries are: position_x, position_y, velocity_x, velocity_y
# The state matrix is composed of the state vectors of all particles
state_matrix = np.zeros((number_of_particles, 2), dtype = float)

#------------------------------------------------------------------------------
# Draw samples from a uniform distribution (initial distribution)
#------------------------------------------------------------------------------
# The basis for using uniform distribution is page 47 of XR-EE-SB 2012:015

# Initialize state data for each particle      
# The initial distribution is set with assumed maximum errors
# For position, an allowance for deviation is given
position_buffer = 0.25
# state_vector[0] = x-position
minimum_position_x = max([starting_position_x - position_buffer, 0])
maximum_position_x = min([starting_position_x + position_buffer, room_width])
state_matrix[:, 0] = np.random.uniform(minimum_position_x, maximum_position_x,
                                       number_of_particles)

# state_vector[1] = y-position
minimum_position_y = max([starting_position_y - position_buffer, 0])
maximum_position_y = min([starting_position_y + position_buffer, room_length])
state_matrix[:, 1] = np.random.uniform(minimum_position_y, maximum_position_y,
                                       number_of_particles)

#------------------------------------------------------------------------------
# Final preparations for main loop
#------------------------------------------------------------------------------
    
# Initialize the AGV
robot = agv.Vehicle(0.024, 2, 0.8, 0.001)
sim_time = 0    

###############################################################################
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------  Main Loop (Iterations) ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
###############################################################################

# Repeat the loop for a given amount of time
print("Start of main loop\n")
sim_duration = 60
count = 0
while sim_time < sim_duration:
    count += 1
    print("Iteration # " + str(count), end = "")
    print("; sim_time = " + "{:.2f}".format(sim_time) + "s")
    robot.setMotor(30, 0)

    if sim_time >= 5 and sim_time < 10:
        robot.setMotor(0, 30)
    if sim_time >= 10 and sim_time < 15:
        robot.setMotor(-20, 0)
    if sim_time >= 15 and sim_time < 30:
        robot.setMotor(10, 30)
    if sim_time >= 30 and sim_time < 40:
        robot.setMotor(-10, 20)
    if sim_time >= 40 and sim_time < 60:
        robot.setMotor(10, -10)
        
#------------------------------------------------------------------------------
# Measurement prediction
#------------------------------------------------------------------------------

    # Reshape the arrays to allow for calculations
    temp_position_x = np.reshape(state_matrix[:, 0], (number_of_particles, 1))
    temp_position_y = np.reshape(state_matrix[:, 1], (number_of_particles, 1))
    
    # Given the positions, the distances to the transmitters can be calculated
    # Transmitter 0 is located at (0, 0) or (0, 0)
    # Transmitter 1 is located at (max_x, 0) or (room_width, 0)
    temp_0 = temp_position_x ** 2 + temp_position_y ** 2
    temp_1 = ((temp_position_x - 
                np.full((number_of_particles, 1), room_width)) ** 2 +
               temp_position_y ** 2)
    
    # Reshape the arrays to allow for calculations
    distance_squared_0 = np.reshape(temp_0, (number_of_particles, 1))
    distance_squared_1 = np.reshape(temp_1, (number_of_particles, 1))
    
    # - (room_width * ones)
    # Use the empirically-determined factor for the inverse-square law
    # signal_strength = factor * (1 / distance ** 2)
    predicted[:, 0] = np.reshape(np.divide(np.full((number_of_particles, 1), 
                                                    factor_0), 
                                           distance_squared_0),
                                 number_of_particles)
    predicted[:, 1] = np.reshape(np.divide(np.full((number_of_particles, 1), 
                                                    factor_1), 
                                           distance_squared_1),
                                 number_of_particles)

#------------------------------------------------------------------------------
# Actual measurement
#------------------------------------------------------------------------------
    
    # Get actual position from simulation
    actual_position_x, actual_position_y = robot.readActual()
    
    # Get IEEE 802.15.4a and encoder values
    # Update the AGV model
    robot.updateModel()
    sim_time += robot.dt
    
    # Get IEEE values at the start
    IEEE_0_start, IEEE_1_start = robot.readIEEE()
    
    # Set time for counter encoder rotations to T seconds (1 second)
    timeout = sim_time + T
    
    # Get the encoder values at the start
    encoder_0_start, encoder_1_start = robot.readEncoder()
    
    # Loop until timeout occurs
    # Measurement is assumed to be quick enough to record increments of 1 or -1 
    while sim_time < timeout:
        robot.updateModel()
        sim_time += robot.dt
    
    # Get encoder values again at the end
    encoder_0_end, encoder_1_end = robot.readEncoder()
    
    # Get IEEE values again at the end
    IEEE_0_end, IEEE_1_end = robot.readIEEE()
    
    # Take the average of signal values across time T
    IEEE_0 = (IEEE_0_start + IEEE_0_end) / 2
    IEEE_1 = (IEEE_1_start + IEEE_1_end) / 2
    
    # Take the difference of the encoder values across time T 
    encoder_0 = encoder_0_end - encoder_0_start
    encoder_1 = encoder_1_end - encoder_1_start
    
    # Update the AGV model
    robot.updateModel()
    sim_time += robot.dt
    
    # NOTE: delete this cluster if IEEE values are signal positions already
    # The simulation library gave positions instead of signal values
    IEEE_position_x, IEEE_position_y = IEEE_0, IEEE_1
    IEEE_0 = factor_0 / (IEEE_position_x ** 2 + IEEE_position_y ** 2)
    IEEE_1 = factor_1 / ((IEEE_position_x - room_width) ** 2 + 
                         IEEE_position_y ** 2)
    
    # Get position if only purely encoder data is used for localization
    encoder_position_x = (encoder_position_x + 
                          encoder_0 * np.pi * diameter * 0.5 / 360)
    encoder_position_y = (encoder_position_y + 
                          encoder_1 * np.pi * diameter * 0.5 / 360)
    
    # Record actual position for plot data
    new_actual = np.array([[actual_position_x, actual_position_y]])
    actual_plot_data = np.append(actual_plot_data, new_actual, axis = 0)

    # Record IEEE position for plot data
    new_IEEE = np.array([[IEEE_position_x, IEEE_position_y]])
    IEEE_plot_data = np.append(IEEE_plot_data, new_IEEE, axis = 0)
    
    # Record encoder position for plot data
    new_encoder = np.array([[encoder_position_x, encoder_position_y]])
    encoder_plot_data = np.append(encoder_plot_data, new_encoder, axis = 0)
    
    # For two pairs of omni-wheels, with one encoder for each pair
    # Each pair is coupled so they move together
    # One pair moves in the x-direction, the other in the y-direction
    measured_velocity_x = (encoder_0 / T) * np.pi * diameter * 0.5 / 360
    measured_velocity_y = (encoder_1 / T) * np.pi * diameter * 0.5 / 360
    
#------------------------------------------------------------------------------
# Modify the weights
#------------------------------------------------------------------------------

    # Randomize measurement noise (white Gaussian noise)
    noise_measurement = np.random.normal(mean, std_dev_measurement,
                                         (number_of_particles, 1))  
    
    # Update the weights based on IEEE 802.15.4a values
    # Take the average of the errors from the two sensors
    # Use the first two columns from the measurement prediction matrix
    IEEE = np.concatenate((IEEE_0 * ones, IEEE_1 * ones), axis = 1)
    difference = np.mean(np.absolute(np.subtract(IEEE, predicted[:, [0,1]])), 
                         axis = 1)
    difference = np.reshape(difference, (number_of_particles, 1))
    # THe weight is proportional to the old weight
    # The weight is proportional to the difference between the reading
    # The weight is proportional to the Z-score of the noise

    weight = np.multiply(np.multiply(weight, difference), 
                              (np.absolute(noise_measurement - mean) / 
                               std_dev_measurement))
    
    # Normalize IEEE 802.15.4a weights 
    weight_total = np.sum(weight)
    weight = weight / weight_total
    
#------------------------------------------------------------------------------
# Output sensor-fused value
#------------------------------------------------------------------------------

    # Reshape the arrays to allow for calculations
    temp_position_x = np.reshape(state_matrix[:, 0], (number_of_particles, 1))
    temp_position_y = np.reshape(state_matrix[:, 1], (number_of_particles, 1))
    
    # The estimated position values are the main output
    # These values are used for the next iteration
    # Get the summation of the element-wise product of values and weights
    estimated_position_x = np.sum(np.multiply(temp_position_x, weight))
    estimated_position_y = np.sum(np.multiply(temp_position_y, weight))
    estimated_velocity_x = measured_velocity_x
    estimated_velocity_y = measured_velocity_y

    # Record estimated position for plot data
    print("\t\t\t\tEstimated x = " + str(estimated_position_x))
    print("\t\t\t\tEstimated y = " + str(estimated_position_y))
    new_estimate = np.array([[estimated_position_x, estimated_position_y]])
    estimated_plot_data = np.append(estimated_plot_data, new_estimate,
                                    axis = 0)
    
#------------------------------------------------------------------------------
# Conduct resampling
#------------------------------------------------------------------------------

    # Resample if the number of effective particles is below a threshold
    # The systematic within residual resampling method is used
    # The basis for this is page 21 of by Murray Pollock's 
    # “Introduction to Particle Filtering” Discussion - Notes
    effective_number_of_particles = 1 / np.sum(np.square(weight))
    if effective_number_of_particles < 0.8 * number_of_particles:
        # Get the integer part and the residual part of the scaled weights
        scaled_weight = np.multiply(np.full((number_of_particles, 1), 
                                             number_of_particles),
                                     weight)
        (number_of_copies, residual) = np.divmod(scaled_weight, 1)
        hi1 = np.sum(number_of_copies)
        normalized_residual = np.divide(residual,
                                        np.full((number_of_particles, 1),
                                                np.sum(residual)))
        cumulative_sum = np.cumsum(normalized_residual)
        # Make sure that the sequence ends with 1 (not more or less)
        cumulative_sum[-1] = 1
        
        residual_selections = number_of_particles - np.sum(number_of_copies)
        hi2 = residual_selections
        if residual_selections > 0:
            offset = np.random.random() / residual_selections
        else:
            offset = 0
        selected_positions = np.linspace(offset, 1 + offset, 
                                         residual_selections.astype(int), 
                                         endpoint = False) 

        particle = 0
        selected = 0
        while selected < residual_selections:
            while selected_positions[selected] < cumulative_sum[particle]:
                number_of_copies[particle] += 1
                selected += 1
                if selected == residual_selections:
                    break
            particle += 1
        
        particle = 0
        entry = 0
        hi3 = np.sum(number_of_copies)
        while particle < number_of_particles:
            for samples in range(int(number_of_copies[particle])):
                resampling_indices[entry] = particle
                entry += 1
            particle += 1
                
        state_matrix_resample = np.take(state_matrix, 
                                        resampling_indices.astype(int))
        weight_resample = np.take(weight, resampling_indices.astype(int))

#------------------------------------------------------------------------------
# Update the particles
#------------------------------------------------------------------------------
    
    # Randomize process noise (white Gaussian noise)
    noise_process = np.random.normal(mean, std_dev_process, 
                                     (number_of_particles, 2))

    # Consolidate the measured velocity into one matrix
    measured_velocity = np.concatenate((np.full((number_of_particles, 1), 
                                        measured_velocity_x), 
                                        np.full((number_of_particles, 1), 
                                        measured_velocity_y)), axis = 1)
    
    # Update the position using position, velocity, and noise
    state_matrix = np.add(state_matrix, 
                          np.multiply(np.add(measured_velocity, noise_process),
                                      np.full((number_of_particles, 2), T))) 
#------------------------------------------------------------------------------
# Not particle filter
#------------------------------------------------------------------------------

    # Use sensor values to get the state
    # Then, refer back to the previous value, putting more weight to it
    # The effect will be a lag in the estimation, cleaning up erratic movements
    state_vector[:] = get_state(IEEE_0, IEEE_1, encoder_0, encoder_1)
    state_vector[0] = ((state_vector[0] + 
                        lookback_scale * (previous_position_x + 
                        previous_velocity_x * T)) / (lookback_scale + 1))
    state_vector[1] = ((state_vector[1] + 
                        lookback_scale * (previous_position_y + 
                        previous_velocity_y * T)) / (lookback_scale + 1))
    
    # The state vector will be the sensor-fused values
    n_estimated_position_x = state_vector[0]
    n_estimated_position_y = state_vector[1]
    n_estimated_velocity_x = state_vector[2]
    n_estimated_velocity_y = state_vector[3]
    
    # Consider estimated state as previous data for the next iteration
    previous_position_x = n_estimated_position_x
    previous_position_y = n_estimated_position_y
    previous_velocity_x = n_estimated_velocity_x
    previous_velocity_y = n_estimated_velocity_y
    
    # Record estimated position for plot data
    n_new_estimate = np.array([[n_estimated_position_x, 
                                n_estimated_position_y]])
    n_estimated_plot_data = np.append(n_estimated_plot_data, n_new_estimate, 
                                      axis = 0)

###############################################################################
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#------------------------ -  Consolidate Results   ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
###############################################################################
    
    progressive_plot = 0
    
    if progressive_plot == 1:
        
#------------------------------------------------------------------------------
# Convert numpy array to list
#------------------------------------------------------------------------------

        actual_vertices_x = np.ndarray.tolist(actual_plot_data[:, 0])
        actual_vertices_y = np.ndarray.tolist(actual_plot_data[:, 1])
        IEEE_vertices_x = np.ndarray.tolist(IEEE_plot_data[:, 0])
        IEEE_vertices_y = np.ndarray.tolist(IEEE_plot_data[:, 1])
        encoder_vertices_x = np.ndarray.tolist(encoder_plot_data[:, 0])
        encoder_vertices_y = np.ndarray.tolist(encoder_plot_data[:, 1])
        estimated_vertices_x = np.ndarray.tolist(estimated_plot_data[:, 0])
        estimated_vertices_y = np.ndarray.tolist(estimated_plot_data[:, 1])
        n_estimated_vertices_x = np.ndarray.tolist(n_estimated_plot_data[:, 0])
        n_estimated_vertices_y = np.ndarray.tolist(n_estimated_plot_data[:, 1])

#------------------------------------------------------------------------------
# Plot the different position data
#------------------------------------------------------------------------------
        
        # Overlapping plots
        plt.plot(actual_vertices_x, actual_vertices_y, 'r')
        plt.plot(IEEE_vertices_x, IEEE_vertices_y, 'b')
        plt.plot(encoder_vertices_x, encoder_vertices_y, 'g')
        plt.plot(estimated_vertices_x, estimated_vertices_y, 'm')
        plt.plot(new_actual[0, 0], new_actual[0, 1], 'r.')
        plt.plot(new_IEEE[0, 0], new_IEEE[0, 1], 'b.')
        plt.plot(new_encoder[0, 0], new_encoder[0, 1], 'g.')
        plt.plot(new_estimate[0, 0], new_estimate[0, 1], 'm.')
#        plt.plot(n_estimated_vertices_x, n_estimated_vertices_y, 'm,-')
        print("\033[H\033[J")
        plt.show()
        
if progressive_plot == 0:
    
#------------------------------------------------------------------------------
# Convert numpy array to list
#------------------------------------------------------------------------------

    actual_vertices_x = np.ndarray.tolist(actual_plot_data[:, 0])
    actual_vertices_y = np.ndarray.tolist(actual_plot_data[:, 1])
    IEEE_vertices_x = np.ndarray.tolist(IEEE_plot_data[:, 0])
    IEEE_vertices_y = np.ndarray.tolist(IEEE_plot_data[:, 1])
    encoder_vertices_x = np.ndarray.tolist(encoder_plot_data[:, 0])
    encoder_vertices_y = np.ndarray.tolist(encoder_plot_data[:, 1])
    estimated_vertices_x = np.ndarray.tolist(estimated_plot_data[:, 0])
    estimated_vertices_y = np.ndarray.tolist(estimated_plot_data[:, 1])
    n_estimated_vertices_x = np.ndarray.tolist(n_estimated_plot_data[:, 0])
    n_estimated_vertices_y = np.ndarray.tolist(n_estimated_plot_data[:, 1])
    
#------------------------------------------------------------------------------
# Plot the different position data
#------------------------------------------------------------------------------

    # Subplot
    plt.subplot(2,3,1)
    plt.plot(actual_vertices_x, actual_vertices_y, 'r')
    plt.title("Actual")
    plt.subplot(2,3,2)
    plt.plot(IEEE_vertices_x, IEEE_vertices_y, 'b')
    plt.title("IEEE")
    plt.subplot(2,3,3)
    plt.plot(encoder_vertices_x, encoder_vertices_y, 'g')
    plt.title("Encoder")
    plt.subplot(2,3,4)
    plt.plot(estimated_vertices_x, estimated_vertices_y, 'm')
    plt.title("Estimate")
    plt.subplot(2,3,6)
    plt.plot(n_estimated_vertices_x, n_estimated_vertices_y, 'm')
    plt.title("Not particle filter")
    plt.tight_layout()
    plt.tight_layout()
    plt.show()
    
    # Multiple plots
    plt.plot(actual_vertices_x, actual_vertices_y, 'r')
    plt.title("Actual")
    plt.show()
    plt.plot(IEEE_vertices_x, IEEE_vertices_y, 'b')
    plt.title("IEEE")
    plt.show()
    plt.plot(encoder_vertices_x, encoder_vertices_y, 'g')
    plt.title("Encoder")
    plt.show()
    plt.plot(estimated_vertices_x, estimated_vertices_y, 'm')
    plt.title("Estimate")
    plt.show()
    plt.plot(n_estimated_vertices_x, n_estimated_vertices_y, 'm')
    plt.title("Not particle filter")
    plt.show()

    # Overlapping plots
    plt.plot(actual_vertices_x, actual_vertices_y, 'r')
    plt.plot(IEEE_vertices_x, IEEE_vertices_y, 'b')
    plt.plot(encoder_vertices_x, encoder_vertices_y, 'g')
    plt.plot(estimated_vertices_x, estimated_vertices_y, 'm')
    plt.plot(n_estimated_vertices_x, n_estimated_vertices_y, 'k')
    plt.show()