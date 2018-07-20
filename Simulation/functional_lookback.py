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

# The main parameter for setting the fusion method     
lookback_scale = 6.20048538418

# Define the time in seconds (s) wherein velocity is measured
T = 0.01

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
    
# Set the orevious and starting states to zero
previous_position_x = 0
previous_position_y = 0
previous_velocity_x = 0
previous_velocity_y = 0
starting_position_x = 0
starting_position_y = 0
starting_velocity_x = 0
starting_velocity_y = 0

# Prepare starting position for encoder position update
encoder_position_x = starting_position_x
encoder_position_y = starting_position_y

# Prepare the arrays to store the position histories for plotting
actual_plot_data = np.zeros((1, 2), dtype = float)
IEEE_plot_data = np.zeros((1, 2), dtype = float)
encoder_plot_data = np.zeros((1, 2), dtype = float)
estimated_plot_data = np.zeros((1, 2), dtype = float)
error_plot_data = np.zeros((1, 7), dtype = float)

#------------------------------------------------------------------------------
# Final preparations for main loop
#------------------------------------------------------------------------------
    
# Initialize the AGV
wheelRadius = diameter/2
mass = 2
friction = 0.8
dt = 0.001
robot = agv.Vehicle(wheelRadius, mass, friction, dt)
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

velocity = 10
radius = 50

while sim_time < sim_duration:
    count += 1
    print("\nIteration # {:d} | ".format(count), end = "")
    print("sim_time = " + "{:.2f}".format(sim_time) + "s", end = "")
    robot.setMotor(10, 10)
    
    if sim_time >= 10:
        vx = velocity * np.cos(velocity*sim_time/radius)
        vy = velocity * np.sin(velocity*sim_time/radius)
        
        robot.setMotor(vx, vy)
        
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
# Functional lookback
#------------------------------------------------------------------------------

    # Use sensor values to get the state
    # Then, refer back to the previous value, putting more weight to it
    # The effect will be a lag in the estimation, cleaning up erratic movements
    (x, y, vx, vy) = get_state(IEEE_0, IEEE_1, encoder_0, encoder_1)
    x = ((x + ((previous_position_x + previous_velocity_x * T) *
                lookback_scale)) / (lookback_scale + 1))
    y = ((y + ((previous_position_y + previous_velocity_y * T) *
                lookback_scale)) / (lookback_scale + 1))
    
    # The state vector will be the sensor-fused values
    estimated_position_x = x
    estimated_position_y = y
    estimated_velocity_x = vx
    estimated_velocity_y = vy
    
    # Consider estimated state as previous data for the next iteration
    previous_position_x = estimated_position_x
    previous_position_y = estimated_position_y
    previous_velocity_x = estimated_velocity_x
    previous_velocity_y = estimated_velocity_y
    
    # Record estimated position for plot data
    new_estimate = np.array([[estimated_position_x, 
                              estimated_position_y]])
    estimated_plot_data = np.append(estimated_plot_data, new_estimate, 
                                    axis = 0)
    
    # Print estimated position for plot data
    print("\t[Estimated x = {:.5f}".format(estimated_position_x), end = "")
    print(" | Estimated y = {:.5f}".format(estimated_position_y), end = "]")
    
    # Calculate the RMS (root-mean-square) error
    IEEE_rms = np.sqrt(np.mean(np.square(
                                         np.subtract(IEEE_plot_data, 
                                                     actual_plot_data))))
    encoder_rms = np.sqrt(np.mean(np.square(
                                            np.subtract(encoder_plot_data, 
                                                        actual_plot_data))))
    estimate_rms = np.sqrt(np.mean(np.square(
                                             np.subtract(estimated_plot_data, 
                                                         actual_plot_data))))
    
    # Calculate the instantaneous error
    IEEE_inst = np.sqrt(np.mean(np.square(
                                          np.subtract(new_IEEE, 
                                                      new_actual))))
    encoder_inst = np.sqrt(np.mean(np.square(
                                             np.subtract(new_encoder, 
                                                         new_actual))))
    estimate_inst = np.sqrt(np.mean(np.square(
                                              np.subtract(new_estimate, 
                                                          new_actual))))
    
    
    # Record progression of error per iteration
    new_error = np.array([[count,
                           IEEE_rms, encoder_rms, estimate_rms,
                           IEEE_inst, encoder_inst, estimate_inst]])
    error_plot_data = np.append(error_plot_data, new_error, axis = 0)

###############################################################################
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#------------------------ -  Consolidate Results   ----------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
###############################################################################
    
    progressive_plot = 0
    iterations_per_update = 50
    
    if progressive_plot == 1:
        
        if np.mod((sim_duration / dt) - count, iterations_per_update) == 0:
            
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
        
            # Overlapping plots
            plt.plot(actual_vertices_x, actual_vertices_y, 'r')
            plt.plot(IEEE_vertices_x, IEEE_vertices_y, 'b')
            plt.plot(encoder_vertices_x, encoder_vertices_y, 'g')
            plt.plot(estimated_vertices_x, estimated_vertices_y, 'm')
            plt.plot(new_actual[0, 0], new_actual[0, 1], 'r.')
            plt.plot(new_IEEE[0, 0], new_IEEE[0, 1], 'b.')
            plt.plot(new_encoder[0, 0], new_encoder[0, 1], 'g.')
            plt.plot(new_estimate[0, 0], new_estimate[0, 1], 'm.')
            plt.axis('equal')
            print("\033[H\033[J")   # Clear the console
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
    
#------------------------------------------------------------------------------
# Plot the different position data
#------------------------------------------------------------------------------
    
    # Overlapping plots
    plt.plot(actual_vertices_x, actual_vertices_y, 'r')
    plt.plot(IEEE_vertices_x, IEEE_vertices_y, 'b')
    plt.plot(encoder_vertices_x, encoder_vertices_y, 'g')
    plt.plot(estimated_vertices_x, estimated_vertices_y, 'm')
    plt.axis('equal')
    plt.show()
    
    # Subplot
    plt.subplot(2,2,1)
    plt.plot(actual_vertices_x, actual_vertices_y, 'r')
    plt.title("Actual")
    plt.axis('equal')
    plt.subplot(2,2,2)
    plt.plot(IEEE_vertices_x, IEEE_vertices_y, 'b')
    plt.title("IEEE")
    plt.axis('equal')
    plt.subplot(2,2,3)
    plt.plot(encoder_vertices_x, encoder_vertices_y, 'g')
    plt.title("Encoder")
    plt.axis('equal')
    plt.subplot(2,2,4)
    plt.plot(estimated_vertices_x, estimated_vertices_y, 'm')
    plt.title("Estimate")
    plt.axis('equal')
    plt.tight_layout(pad=0.25, w_pad=1.0, h_pad=1.0)
    plt.show()
    
#    # Multiple plots
#    plt.plot(actual_vertices_x, actual_vertices_y, 'r')
#    plt.title("Actual")
#    plt.axis('equal')
#    plt.show()
#    plt.plot(IEEE_vertices_x, IEEE_vertices_y, 'b')
#    plt.title("IEEE")
#    plt.axis('equal')
#    plt.show()
#    plt.plot(encoder_vertices_x, encoder_vertices_y, 'g')
#    plt.title("Encoder")
#    plt.axis('equal')
#    plt.show()
#    plt.plot(estimated_vertices_x, estimated_vertices_y, 'm')
#    plt.title("Estimate")
#    plt.axis('equal')
#    plt.show()
    
#------------------------------------------------------------------------------
# Plot the error
#------------------------------------------------------------------------------

iteration_values = np.ndarray.tolist(error_plot_data[:, 0])
IEEE_rms_values = np.ndarray.tolist(error_plot_data[:, 1])
encoder_rms_values = np.ndarray.tolist(error_plot_data[:, 2])
estimate_rms_values = np.ndarray.tolist(error_plot_data[:, 3])
IEEE_inst_values = np.ndarray.tolist(error_plot_data[:, 4])
encoder_inst_values = np.ndarray.tolist(error_plot_data[:, 5])
estimate_inst_values = np.ndarray.tolist(error_plot_data[:, 6])

# Plot error progression
plt.subplot(2,2,1)
plt.plot(iteration_values, IEEE_inst_values, 'r')
plt.title("Instantaneous IEEE error")
plt.subplot(2,2,2)
plt.plot(iteration_values, encoder_inst_values, 'b')
plt.title("Instantaneous encoder error")
plt.subplot(2,2,3)
plt.plot(iteration_values, estimate_inst_values, 'g')
plt.title("Instantaneous estimate error")
plt.subplot(2,2,4)
plt.plot(iteration_values, IEEE_rms_values, 'r')
plt.plot(iteration_values, encoder_rms_values, 'b')
plt.plot(iteration_values, estimate_rms_values, 'g')
plt.title("RMS error")
plt.tight_layout(pad=0.25, w_pad=1.0, h_pad=1.0)
plt.show()

# Print error values
print("\n   [Final RMS Error]")
print("IEEE     = " + str('{:7.4f}'.format(IEEE_rms_values[-1])))
print("Encoder  = " + str('{:7.4f}'.format(encoder_rms_values[-1])))
print("Estimate = " + str('{:7.4f}'.format(estimate_rms_values[-1])))