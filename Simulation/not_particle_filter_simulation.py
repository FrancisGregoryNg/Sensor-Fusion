import numpy as np
import matplotlib.pyplot as plt
import agv_library as agv

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------   Function Definitions  ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Used for gettting the sensor measurements
def get_measurements():
    global sim_time
    robot.updateModel()
    sim_time += robot.dt
    
    # Get IEEE at the start
    IEEE_0_start, IEEE_1_start = robot.readIEEE()
    
    # Set time for counter encoder rotations to T seconds (1 second)
    timeout = sim_time + T
    
    # Initialize the encoder
    encoder_0_start, encoder_1_start = robot.readEncoder()
    
    # Loop until timeout occurs
    # Measurement is assumed to be quick enough to record increments of 1 or -1 
    while sim_time < timeout:
        robot.updateModel()
        sim_time += robot.dt
    
    encoder_0_end, encoder_1_end = robot.readEncoder()
    
    # Get IEEE again at the end
    IEEE_0_end, IEEE_1_end = robot.readIEEE()
    
    IEEE_0 = (IEEE_0_start + IEEE_0_end) / 2
    IEEE_1 = (IEEE_1_start + IEEE_1_end) / 2
    
    encoder_0 = encoder_0_end - encoder_0_start
    encoder_1 = encoder_1_end - encoder_1_start
    
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

    return IEEE_0, IEEE_1, encoder_0, encoder_1

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
    position_x = prior_x + encoder_0 * np.pi * diameter * 0.5 / 360
    position_y = prior_y + encoder_1 * np.pi * diameter * 0.5 / 360
    return position_x, position_y

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------      Initialization     ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
robot = agv.Vehicle(0.024, 2, 0.8, 0.001)

# Initialize simulation time
sim_time = 0

# Initialize starting state and previous state to zero
starting_position_x = 0
starting_position_y = 0
starting_velocity_x = 0
starting_velocity_y = 0
previous_position_x = 0
previous_position_y = 0
previous_velocity_x = 0
previous_velocity_y = 0

# Prepare the arrays to store the position histories for plotting
actual_plot_data = np.zeros((1, 2), dtype = float)
IEEE_plot_data = np.zeros((1, 2), dtype = float)
encoder_plot_data = np.zeros((1, 2), dtype = float)
estimated_plot_data = np.zeros((1, 2), dtype = float)
    
# Initialize the state vector
state_vector = np.zeros(4, dtype = float)

#------------------------------------------------------------------------------
# Values for encoder
#------------------------------------------------------------------------------

# Prepare starting position for encoder position update
encoder_position_x = starting_position_x
encoder_position_y = starting_position_y

# Define the diameter of each omni-wheel is in meters (m)
diameter = 0.024 * 2

# Define the time in seconds (s) wherein velocity is measured
T = 0.1

#------------------------------------------------------------------------------
# Values for IEEE 802.15.4a
#------------------------------------------------------------------------------

# Define the proportionality factors for signal strength
factor_0 = 1
factor_1 = 1

# Define the room dimensions (width, length) correspond to (x, y)
# The origin starts at a selected corner of the room
# The measurement unit is in meters (m)
room_width = 5
room_length = 5

#------------------------------------------------------------------------------
# Actual measurement
#------------------------------------------------------------------------------

# Get IEEE 802.15.4a and encoder values
IEEE_0, IEEE_1, encoder_0, encoder_1 = get_measurements()
    
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------  Main Loop (Iterations) ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
print("Start of main loop\n")
# Repeat the loop for a given amount of time

iteration = 0

while sim_time < 20:
    robot.setMotor(30, 0)
    iteration += 1
    print("Iteration # " + str(iteration), end = "")
    print("; sim_time = " + "{:.2f}".format(sim_time) + "s")  
    robot.setMotor(30, 0)
    set_1 = 0
    set_2 = 0
    set_3 = 0
    if sim_time > 5 and set_1 == 0:
        robot.setMotor(0, 30)
        set_1 = 1
    if sim_time > 10 and set_2 == 0:
        robot.setMotor(-30, 0)
        set_2 = 1
    if sim_time > 15 and set_3 == 0:
        robot.setMotor(0, -30)
        set_3 = 1
    
#------------------------------------------------------------------------------
# Actual measurement
#------------------------------------------------------------------------------

    # Get actual position from simulation
    actual_position_x, actual_position_y = robot.readActual()
    
    # Actual position for plot data
    new_actual = np.array([[actual_position_x, actual_position_y]])
    actual_plot_data = np.append(actual_plot_data, new_actual, axis = 0)
    
    # Get IEEE 802.15.4a and encoder values
    IEEE_0, IEEE_1, encoder_0, encoder_1 = get_measurements()
    
    # Get position if only purely IEEE 802.15.4a data is used for localization
    IEEE_position_x, IEEE_position_y = IEEE_0, IEEE_1
    
    # Get the actual measurements because simulation gave positions
    IEEE_0, IEEE_1 = get_IEEE(IEEE_0, IEEE_1)
    
    # Calculate the IEEE position
    # Unneeded because simulation provides position rather than signal strength
    # Might be needed in the future when signal strength is given instead
    IEEE_position_x, IEEE_position_y = get_IEEE_plot_data(IEEE_0, IEEE_1)
    
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
# Output sensor-fused value
#------------------------------------------------------------------------------

    state_vector[:] = get_state(IEEE_0, IEEE_1, encoder_0, encoder_1)
    state_vector[0] = ((state_vector[0] + 
                        5 * (previous_position_x + 
                        previous_velocity_x * T)) / 6)
    state_vector[1] = ((state_vector[1] + 
                        5 * (previous_position_y + 
                        previous_velocity_y * T)) / 6)
    
    # The state vector will be the sensor-fused values
    estimated_position_x = state_vector[0]
    estimated_position_y = state_vector[1]
    estimated_velocity_x = state_vector[2]
    estimated_velocity_y = state_vector[3]
    
    # Consider estimated state as previous data for the next iteration
    previous_position_x = estimated_position_x
    previous_position_y = estimated_position_y
    previous_velocity_x = estimated_velocity_x
    previous_velocity_y = estimated_velocity_y

    print("\t\t\t\tEstimated x = " + str(estimated_position_x))
    print("\t\t\t\tEstimated y = " + str(estimated_position_y))
    
    # Record estimated position for plot data
    new_estimate = np.array([[estimated_position_x, estimated_position_y]])
    estimated_plot_data = np.append(estimated_plot_data, new_estimate, 
                                    axis = 0)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#------------------------ -  Consolidate Results   ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

#------------------------------------------------------------------------------
# Plot the different position data
#------------------------------------------------------------------------------

# Subplot
plt.subplot(2,2,1)
plt.plot(actual_vertices_x, actual_vertices_y, 'r,-')
plt.title("Actual")
plt.subplot(2,2,2)
plt.plot(IEEE_vertices_x, IEEE_vertices_y, 'b,-')
plt.title("IEEE")
plt.subplot(2,2,3)
plt.plot(encoder_vertices_x, encoder_vertices_y, 'g,-')
plt.title("Encoder")
plt.subplot(2,2,4)
plt.plot(estimated_vertices_x, estimated_vertices_y, 'm,-')
plt.title("Estimate")
plt.tight_layout()
plt.show()

# Multiple plots
plt.plot(actual_vertices_x, actual_vertices_y, 'r,-')
plt.title("Actual")
plt.show()
plt.plot(IEEE_vertices_x, IEEE_vertices_y, 'b,-')
plt.title("IEEE")
plt.show()
plt.plot(encoder_vertices_x, encoder_vertices_y, 'g,-')
plt.title("Encoder")
plt.show()
plt.plot(estimated_vertices_x, estimated_vertices_y, 'm,-')
plt.title("Estimate")
plt.show()

# Overlapping plots
plt.plot(actual_vertices_x, actual_vertices_y, 'r,-')
plt.plot(IEEE_vertices_x, IEEE_vertices_y, 'b,-')
plt.plot(encoder_vertices_x, encoder_vertices_y, 'g,-')
plt.plot(estimated_vertices_x, estimated_vertices_y, 'm,-')
plt.show()