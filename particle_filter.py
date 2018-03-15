import matplotlib.pyplot as plt
import numpy as np
from numpy.random import uniform
import csv

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------   Function Definitions  -------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Used for getting the position using the IEEE 802.15.4a values
def get_position_IEEE(IEEE_1, IEEE_2):
    # find the most similar signal signature from database
    database
    x = 0
    y = 0
    return x, y

# Used for getting the position using the three encoder values
def get_position_encoder(encoder_1, encoder_2, encoder_3):
    # formulate after determining the AGV configuration
    x = 0
    y = 0
    return x, y

# Used for converting position data from encoder into velocity (per component)
def get_velocity(position_previous, position_present, sensor_frequency):
    velocity = (position_present - position_previous) * sensor_frequency
    return velocity

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------      Initialization     -------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#---------------------------------------------------------------------------
# Set up database
#---------------------------------------------------------------------------

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

#---------------------------------------------------------------------------
# Set values
#---------------------------------------------------------------------------

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

#---------------------------------------------------------------------------
# Draw samples from a uniform distribution (initial distribution)
#---------------------------------------------------------------------------

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

#---------------------------------------------------------------------------
# Gather sensor values
#---------------------------------------------------------------------------

# IEEE 802.15.4a signals are received from two transmitters 
# it is assumed that multiplexing is done
# the sensor frequency must be at least twice that of the unified frequency
IEEE_0 = 0 # pin readout from pin A
IEEE_1 = 0 # pin readout from pin A at an offest of < 0.5 * sensor_frequency

encoder_0 = 0 # pin readout from pin B
encoder_1 = 0 # pin readout from pin C
encoder_2 = 0 # pin readout from pin D

# from IEEE 802.15.4a input
position_x, position_y = get_position_IEEE(IEEE_0, IEEE_1)

# from encoder input
x_present, y_present = get_position_encoder(encoder_0, encoder_1, encoder_2)

# start with zero velocity due to lack of data as the system just initialized
velocity_x = 0
velocity_y = 0

# optional plot of particle position
plt.scatter(position_x, position_y, marker = ',', edgecolor = 'r', s = 1)
plt.show()

#---------------------------------------------------------------------------
# Compute the weights
#---------------------------------------------------------------------------



#---------------------------------------------------------------------------
# Normalize the weights
#---------------------------------------------------------------------------



#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------  Main Loop (Iterations) -------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#---------------------------------------------------------------------------
# Use importance distribution
#---------------------------------------------------------------------------



#---------------------------------------------------------------------------
# Gather sensor values
#---------------------------------------------------------------------------

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


#---------------------------------------------------------------------------
# Update weights
#---------------------------------------------------------------------------



#---------------------------------------------------------------------------
# Normalize weights
#---------------------------------------------------------------------------



#---------------------------------------------------------------------------
# Output sensor-fused value
#---------------------------------------------------------------------------



#---------------------------------------------------------------------------
# Conduct resampling
#---------------------------------------------------------------------------


