import numpy as np
import matplotlib.pyplot as plt
import agv_library as agv

number_of_particles = 1000

starting_position_x = 0
starting_position_y = 0
starting_velocity_x = 0
starting_velocity_y = 0

room_width = 5
room_length = 5

state_matrix = np.zeros((number_of_particles, 2), dtype = float)

position_buffer = 0.5
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
