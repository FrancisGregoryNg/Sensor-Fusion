import numpy as np
import matplotlib.pyplot as plt
import agv_library as agv

###############################################################################
###############################################################################
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------   Function Definitions  ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
###############################################################################
###############################################################################

# Used for gettting the sensor measurements
def get_measurements():
    global sim_time
    
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

    # Return values
    return IEEE_0, IEEE_1, encoder_0, encoder_1

# Get the position using the IEEE 802.15.4a values
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
    
    # Return values
    return IEEE_0, IEEE_1

# Get the position using the IEEE 802.15.4a values
def get_IEEE_plot_data(IEEE_0, IEEE_1):
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
    
    # Return values
    return position_x, position_y

# Get the position using the encoder values  
def get_encoder_plot_data(encoder_0, encoder_1, prior_x, prior_y):
    # The prior position is purely from the previous iteration of this function
    position_x = prior_x + encoder_0 * np.pi * diameter * 0.5 / 360
    position_y = prior_y + encoder_1 * np.pi * diameter * 0.5 / 360
    
    # Return values
    return position_x, position_y

###############################################################################
###############################################################################    
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------      Initialization     ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
###############################################################################
###############################################################################

swarm_particles = 100
max_std_dev = 10
swarm_position = np.random.uniform(0, max_std_dev, (swarm_particles, 3))
swarm_velocity = np.random.uniform(-0.1, 0.1, (swarm_particles, 3))
local_best_position = np.copy(swarm_position)
local_best_velocity = np.copy(swarm_velocity)
global_best_position = swarm_position[0, :]
global_best_velocity = swarm_velocity[0, :]
local_best_cost = np.full((swarm_particles), np.inf)
global_best_cost = np.inf

epoch = 0
limit = 150
inertial_coefficient = 1

while epoch < limit:
    epoch += 1
    print("Epoch = ", epoch)
    for element in range(swarm_particles):
        #------------------------------------------------------------------------------
        # Define the parameters
        #------------------------------------------------------------------------------
            
        # number of particles is set heuristically
        number_of_particles = 1000
        
        # Define the factor of contribution of previous state campared to new data
        # The time update of the state will not simply neglect the previous estimation
        lookback_scale = 0.75
        
        # Set the lookback scale for the other method
        n_lookback_scale = lookback_scale
        
        # Define the covariance of process noise and measurement noise
        std_dev_process = swarm_position[element, 0] 
        std_dev_measurement_IEEE = swarm_position[element, 1] 
        std_dev_measurement_encoder = swarm_position[element, 2] 
        
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
        
        # Set the previous state to zero
        previous_position_x = 0
        previous_position_y = 0
        previous_velocity_x = 0
        previous_velocity_y = 0
        
        # Set the previous state to zero (for the non-particle filter)
        n_previous_position_x = 0
        n_previous_position_y = 0
        n_previous_velocity_x = 0
        n_previous_velocity_y = 0
        
        # Create an array of ones for use in computations
        ones = np.ones((number_of_particles, 1), dtype = float)
        
        # Prepare the matrix for storing predicted measurements
        # IEEE_0, IEEE_1, encoder_0, encoder_1
        predicted = np.zeros((number_of_particles, 4), dtype = float)
        
        # Prepare the matrix for storing white Gaussian noise values
        noise_measurement = np.zeros((number_of_particles, 1), dtype = float)
        noise_process = np.zeros((number_of_particles, 2), dtype = float)
        
        # Initialize the vector for direct sensor state
        direct_sensor = np.zeros(4, dtype = float)
        
        # Prepare the arrays to store the position histories for plotting
        actual_plot_data = np.zeros((1, 2), dtype = float)
        IEEE_plot_data = np.zeros((1, 2), dtype = float)
        encoder_plot_data = np.zeros((1, 2), dtype = float)
        estimated_plot_data = np.zeros((1, 2), dtype = float)
        
        # Prepare the array to store the position histor for plotting of other method
        n_estimated_plot_data = np.zeros((1, 2), dtype = float)
        
        # Initialize the weights as equal for all particles
        weight = np.ones((number_of_particles, 1), dtype = float)
        weight = weight / number_of_particles
        
        # Initialize the temporary weights due to IEEE 802.15.4a and encoder values
        weight_IEEE = np.zeros((number_of_particles, 1), dtype = float)
        weight_encoder = np.zeros((number_of_particles, 1), dtype = float)
        
        # Initialize the temporary arrays for resampling
        resample = np.zeros((number_of_particles, 1), dtype = int)
        weight_resample = np.zeros((number_of_particles, 1), dtype = float)
        state_matrix_resample = np.zeros((number_of_particles, 4), dtype = float)
        
        # Initialize the state matrix with zeros
        # Use the constant velocity dynamic model
        # Entries are: position_x, position_y, velocity_x, velocity_y
        # The state matrix is composed of the state vectors of all particles
        state_matrix = np.zeros((number_of_particles, 4), dtype = float)
           
        # Initialize the state vector (for the non-particle filter)
        state_vector = np.zeros(4, dtype = float)
        
        #------------------------------------------------------------------------------
        # Draw samples from a uniform distribution (initial distribution)
        #------------------------------------------------------------------------------
        # The basis for using uniform distribution is page 47 of XR-EE-SB 2012:015
        
        # Initialize state data for each particle      
        # The initial distribution is set with assumed maximum errors
        # For position, an allowance for deviation is given
        position_buffer = 0.025
        
        # For velocity, an allowance for deviation is given
        velocity_buffer = 0.025
        
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
        
        # state_vector[2] = x-velocity
        minimum_velocity_x = starting_velocity_x - velocity_buffer
        maximum_velocity_x = starting_velocity_x + velocity_buffer
        state_matrix[:, 2] = np.random.uniform(minimum_velocity_x, maximum_velocity_x,
                                               number_of_particles)
        
         # state_vector[3] = y-velocity
        minimum_velocity_y = starting_velocity_y - velocity_buffer
        maximum_velocity_y = starting_velocity_y + velocity_buffer
        state_matrix[:, 3] = np.random.uniform(minimum_velocity_y, maximum_velocity_y,
                                               number_of_particles)
        
        #------------------------------------------------------------------------------
        # Final preparations for main loop
        #------------------------------------------------------------------------------
            
        # Initialize the AGV
        robot = agv.Vehicle(0.024, 2, 0.8, 0.001)
        sim_time = 0    
        
        # Get IEEE 802.15.4a and encoder values
        IEEE_0, IEEE_1, encoder_0, encoder_1 = get_measurements()
        
        ###############################################################################
        ###############################################################################
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        #-------------------------  Main Loop (Iterations) ----------------------------
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        ###############################################################################
        ###############################################################################
        
        # Repeat the loop for a given amount of time
# =============================================================================
#         print("Start of main loop\n")
# =============================================================================
        sim_duration = 20
        count = 0
        while sim_time < sim_duration:
            count += 1
# =============================================================================
#             print("Iteration # " + str(count), end = "")
#             print("; sim_time = " + "{:.2f}".format(sim_time) + "s")  
# =============================================================================
            robot.setMotor(30, 0)
            set_1 = 0
            set_2 = 0
            set_3 = 0
            if sim_time > 0.25 * sim_duration and set_1 == 0:
                robot.setMotor(0, 30)
                set_1 = 1
            if sim_time > 0.50 * sim_duration and set_2 == 0:
                robot.setMotor(-30, 0)
                set_2 = 1
            if sim_time > 0.75 * sim_duration and set_3 == 0:
                robot.setMotor(0, -30)
                set_3 = 1
                
        #------------------------------------------------------------------------------
        # Measurement prediction
        #------------------------------------------------------------------------------
            
            for particle in range(number_of_particles):
                # predict IEEE_0, IEEE_1, encoder_0, encoder_1
                predicted[particle] = predict_measurements(state_matrix[particle][0],
                                                           state_matrix[particle][1],
                                                           state_matrix[particle][2],
                                                           state_matrix[particle][3])
            
        #------------------------------------------------------------------------------
        # Actual measurement
        #------------------------------------------------------------------------------
        
            # Get actual position from simulation
            actual_position_x, actual_position_y = robot.readActual()
            
            # Get IEEE 802.15.4a and encoder values
            IEEE_0, IEEE_1, encoder_0, encoder_1 = get_measurements()
            
            # NOTE: delete this cluster if IEEE values are signal positions already
            # The simulation library gave positions instead of signal values
            IEEE_position_x, IEEE_position_y = IEEE_0, IEEE_1
            IEEE_0, IEEE_1 = get_IEEE(IEEE_position_x, IEEE_position_y)
            
            # Get position if only purely IEEE data is used for localization
            # Unneeded because simulation provides position rather than signal strength
            # Might be needed in the future when signal strength is given instead
            IEEE_position_x, IEEE_position_y = get_IEEE_plot_data(IEEE_0, IEEE_1)
            
            # Get position if only purely encoder data is used for localization
            (encoder_position_x,
             encoder_position_y) = get_encoder_plot_data(encoder_0, encoder_1, 
                                                         encoder_position_x,
                                                         encoder_position_y)
            
            # Record actual position for plot data
            new_actual = np.array([[actual_position_x, actual_position_y]])
            actual_plot_data = np.append(actual_plot_data, new_actual, axis = 0)
        
            # Record IEEE position for plot data
            new_IEEE = np.array([[IEEE_position_x, IEEE_position_y]])
            IEEE_plot_data = np.append(IEEE_plot_data, new_IEEE, axis = 0)
            
            # Record encoder position for plot data
            new_encoder = np.array([[encoder_position_x, encoder_position_y]])
            encoder_plot_data = np.append(encoder_plot_data, new_encoder, axis = 0)
            
        #------------------------------------------------------------------------------
        # Modify the weights
        #------------------------------------------------------------------------------
        
            # Randomize measurement noise (white Gaussian noise)
            noise_measurement = np.random.normal(mean, std_dev_measurement_IEEE,
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
            weight_IEEE = np.multiply(np.multiply(weight, difference), 
                                      ((noise_measurement - mean) / 
                                       std_dev_measurement_IEEE))
                
            # Normalize IEEE 802.15.4a weights 
            weight_total = np.sum(weight_IEEE)
            weight_IEEE = weight_IEEE / weight_total
        
            # Randomize measurement noise (white Gaussian noise)
            noise_measurement = np.random.normal(mean, std_dev_measurement_encoder,
                                                 (number_of_particles, 1))  
            
            # Update the weights based on encoder values
            # Take the average of the errors from the two sensors
            # Use the last two columns from the measurement prediction matrix
            encoder = np.concatenate((encoder_0 * ones, encoder_1 * ones), axis = 1)
            difference = np.mean(np.absolute(np.subtract(encoder, predicted[:,[2,3]])), 
                                 axis = 1) 
            difference = np.reshape(difference, (number_of_particles, 1))  

            # THe weight is proportional to the old weight
            # The weight is proportional to the difference between the reading
            # The weight is proportional to the Z-score of the noise
            weight_encoder = np.multiply(np.multiply(weight, difference),
                                         (1/np.absolute(noise_measurement)))
                 
            # Normalize encoder weights
            weight_total = np.sum(weight_encoder)
            weight_encoder = weight_encoder / weight_total
            
        #------------------------------------------------------------------------------
        # Output sensor-fused value
        #------------------------------------------------------------------------------
        
            # Reshape the arrays to allow for calculations
            temp_position_x = np.reshape(state_matrix[:, 0], (number_of_particles, 1))
            temp_position_y = np.reshape(state_matrix[:, 1], (number_of_particles, 1))
            temp_velocity_x = np.reshape(state_matrix[:, 2], (number_of_particles, 1))
            temp_velocity_y = np.reshape(state_matrix[:, 3], (number_of_particles, 1))
            
            # The estimated position values are the main output
            # These values are used for the next iteration
            # Get the summation of the element-wise product of values and weights
            estimated_position_x = np.sum(np.multiply(temp_position_x, weight_IEEE))
            estimated_position_y = np.sum(np.multiply(temp_position_y, weight_IEEE))
            estimated_velocity_x = np.sum(np.multiply(temp_velocity_x, weight_encoder))
            estimated_velocity_y = np.sum(np.multiply(temp_velocity_y, weight_encoder))
            
            # Update the previous iteration values
            previous_position_x = estimated_position_x
            previous_position_y = estimated_position_y
            previous_velocity_x = estimated_velocity_x
            previous_velocity_y = estimated_velocity_y
        
            # Record estimated position for plot data
# =============================================================================
#             print("\t\t\t\tEstimated x = " + str(estimated_position_x))
#             print("\t\t\t\tEstimated y = " + str(estimated_position_y))
# =============================================================================
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
            effective_number_of_particles = 1 / np.sum(np.power(weight, 2))
            if effective_number_of_particles < 0.8 * number_of_particles:
                # Get the integer part and the residual part of the scaled weights
                (number_of_copies, residual) = np.divmod(number_of_particles * weight, 
                                                         1)
        
                # Select copies by taking note of the particle index
                # Base the selection on the integer part of scaled weights
                selected = 0
                for particle in range(number_of_particles):
                    for copy in range(int(number_of_copies[particle])):
                        resample[selected] = particle
                        selected += 1
                        
                # Get the needed values for the resampling method
                residual = residual / np.sum(residual)
                cumulative_sum = np.cumsum(residual)
                divisions = number_of_particles - np.sum(number_of_copies)
                positions = (np.linspace(0, 1, divisions, endpoint = False) +
                             (np.random.random() / divisions))
                
                # Make sure that the sequence ends with 1 (not more or less)
                cumulative_sum[-1] = 1
                
                # Select copies based on systematic strategy on the residual
                current_division = 0
                for particle in range(number_of_particles):
                    if positions[current_division] <= cumulative_sum[particle]:
                        resample[selected] = particle
                        selected += 1
                        current_division +=1
                        # NOTE: patch-up solution, try to resolve
                        if selected == number_of_particles:
                            break
                        
                # Update the state matrix
                # Take the elements according to the chosen resampling indices
                state_matrix_resample = state_matrix[np.ndarray.tolist(resample)]
                state_matrix_resample = np.reshape(state_matrix_resample, 
                                                   (number_of_particles, 4))
                state_matrix = state_matrix_resample
                
                # Update the weights
                weight_resample = np.take(weight, resample)
                weight = weight_resample
        
        #------------------------------------------------------------------------------
        # Update the particles
        #------------------------------------------------------------------------------
            # Update the state of the particle based on noise and previous state
            # The basis for this is page 47 of 978-1580536318/158053631X
            
            # Randomize process noise (white Gaussian noise)
            noise_process = np.random.normal(mean, std_dev_process, 
                                             (number_of_particles, 2))
        
            # Update the position using position, velocity, and acceleration (noise)
            state_matrix[:, 0] = (state_matrix[:, 0] + 
                                  T * state_matrix[:, 2] + 
                                  0.5 * (T ** 2) * noise_process[:, 0])
            state_matrix[:, 1] = (state_matrix[:, 1] + 
                                  T * state_matrix[:, 3] + 
                                  0.5 * (T ** 2) * noise_process[:, 1])
            
            # Update the velocity using velocity and acceleration (noise)
            state_matrix[:, 2] = (state_matrix[:, 2] + 
                                  T * noise_process[:, 0])
            state_matrix[:, 3] = (state_matrix[:, 3] + 
                                  T * noise_process[:, 1])
            
            # Modify results based on looking back at the previous estimation
            state_matrix[:, 0] = ((state_matrix[:, 0] +
                                   lookback_scale * (previous_position_x +
                                   previous_velocity_x * T)) / (lookback_scale + 1))
            state_matrix[:, 1] = ((state_matrix[:, 1] +
                                   lookback_scale * (previous_position_y +
                                   previous_velocity_y * T)) / (lookback_scale + 1))
                                       
        #------------------------------------------------------------------------------
        # Not particle filter
        #------------------------------------------------------------------------------
        
            # Use sensor values to get the state
            # Then, refer back to the previous value, putting more weight to it
            # The effect will be a lag in the estimation, cleaning up erratic movements
            state_vector[:] = get_state(IEEE_0, IEEE_1, encoder_0, encoder_1)
            state_vector[0] = ((state_vector[0] + 
                                n_lookback_scale * (n_previous_position_x + 
                                n_previous_velocity_x * T)) / (n_lookback_scale + 1))
            state_vector[1] = ((state_vector[1] + 
                                n_lookback_scale * (n_previous_position_y + 
                                n_previous_velocity_y * T)) / (n_lookback_scale + 1))
            
            # The state vector will be the sensor-fused values
            n_estimated_position_x = state_vector[0]
            n_estimated_position_y = state_vector[1]
            n_estimated_velocity_x = state_vector[2]
            n_estimated_velocity_y = state_vector[3]
            
            # Consider estimated state as previous data for the next iteration
            n_previous_position_x = n_estimated_position_x
            n_previous_position_y = n_estimated_position_y
            n_previous_velocity_x = n_estimated_velocity_x
            n_previous_velocity_y = n_estimated_velocity_y
            
            # Record estimated position for plot data
            n_new_estimate = np.array([[n_estimated_position_x, 
                                        n_estimated_position_y]])
            n_estimated_plot_data = np.append(n_estimated_plot_data, n_new_estimate, 
                                              axis = 0)
            
        ###############################################################################
        ###############################################################################
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        #------------------------ -  Consolidate Results   ----------------------------
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        ###############################################################################
        ###############################################################################
            
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
        
# =============================================================================
#         # Subplot
#         plt.subplot(2,3,1)
#         plt.plot(actual_vertices_x, actual_vertices_y, 'r,-')
#         plt.title("Actual")
#         plt.subplot(2,3,2)
#         plt.plot(IEEE_vertices_x, IEEE_vertices_y, 'b,-')
#         plt.title("IEEE")
#         plt.subplot(2,3,3)
#         plt.plot(encoder_vertices_x, encoder_vertices_y, 'g,-')
#         plt.title("Encoder")
#         plt.subplot(2,3,4)
#         plt.plot(estimated_vertices_x, estimated_vertices_y, 'm,-')
#         plt.title("Estimate")
#         plt.subplot(2,3,6)
#         plt.plot(n_estimated_vertices_x, n_estimated_vertices_y, 'm,-')
#         plt.title("Not particle filter")
#         plt.tight_layout()
#         plt.show()
#         
#         # Multiple plots
#         plt.plot(actual_vertices_x, actual_vertices_y, 'r,-')
#         plt.title("Actual")
#         plt.show()
#         plt.plot(IEEE_vertices_x, IEEE_vertices_y, 'b,-')
#         plt.title("IEEE")
#         plt.show()
#         plt.plot(encoder_vertices_x, encoder_vertices_y, 'g,-')
#         plt.title("Encoder")
#         plt.show()
#         plt.plot(estimated_vertices_x, estimated_vertices_y, 'm,-')
#         plt.title("Estimate")
#         plt.show()
#         plt.plot(n_estimated_vertices_x, n_estimated_vertices_y, 'm,-')
#         plt.title("Not particle filter")
#         plt.show()
#         
#         # Overlapping plots
#         plt.plot(actual_vertices_x, actual_vertices_y, 'r,-')
#         plt.plot(IEEE_vertices_x, IEEE_vertices_y, 'b,-')
#         plt.plot(encoder_vertices_x, encoder_vertices_y, 'g,-')
#         plt.plot(estimated_vertices_x, estimated_vertices_y, 'm,-')
#         plt.plot(n_estimated_vertices_x, n_estimated_vertices_y, 'm,-')
#         plt.show()
# =============================================================================
    
        current_cost = np.sqrt(np.mean(np.square(np.subtract(estimated_plot_data,
                                                             n_estimated_plot_data))))
    
        if current_cost < local_best_cost[element]:
            print("Epoch = ", epoch, end='')
            print("; Updated local best for particle [{}]".format(element),
                  "from:", local_best_cost[element], end='')
            local_best_cost[element] = current_cost
            local_best_position[element, :] = swarm_position[element, :] 
            print(" | to:", local_best_cost[element])
            if local_best_cost[element] < global_best_cost:
                print("Epoch = ", epoch, end='')
                print("; Updated global best from:", global_best_cost, end='')
                global_best_cost = local_best_cost[element]
                global_best_position = swarm_position[element, :]
                global_best_velocity = swarm_velocity[element, :]
                print(" | to:", global_best_cost)
        
    if epoch > 0.20 * limit:
        inertial_coefficient = 0.8
    if epoch > 0.40 * limit:
        inertial_coefficient = 0.7
    if epoch > 0.60 * limit:
        inertial_coefficient = 0.6
    if epoch > 0.80 * limit:
        inertial_coefficient = 0.5
        
    swarm_position = np.add(swarm_position, swarm_velocity)
    swarm_position = np.clip(swarm_position, 0, max_std_dev)
    swarm_velocity = np.add(inertial_coefficient * swarm_velocity, 
                            2 * np.multiply(np.random.rand(swarm_particles, 3), 
                                            np.subtract(local_best_position,
                                                        swarm_position)),
                            2 * np.multiply(np.random.rand(swarm_particles, 3), 
                                            np.subtract(global_best_position,
                                                        swarm_position)))

print("best_std_dev_process = ", global_best_position[0])
print("best_std_dev_measurement_IEEE = ", global_best_position[1])
print("best_std_dev_measurement_encoder = ", global_best_position[2])
