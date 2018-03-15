import csv

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------      Initialization     -------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Set the size of the room (must be the same with particle_filter.py)
room_width = 5000
room_length = 5000

# The balance between precision and difficulty is determined heuristically
grid_precision = 100

# The size is determined inclusive of the origin which is located in a corner
size_of_x = 1 + room_width/grid_precision
size_of_y = 1 + room_length/grid_precision

# Calculate the number of datapoints for use in the signal mapping
number_of_datapoints = size_of_x * size_of_y

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------      Grid Locations     -------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Start with the 
x, y = 0, 0

# Monitor the reversals because the progression is done in a snake-like manner
direction = 1
counter = 0

# Set the grid locations
with open('IEEE_matching_locations.csv') as locations:
    coordinates = ['x', 'y']
    write_locations = csv.DictWriter(locations, fieldnames = coordinates)
    write_locations.writeheader()

# Make the grid locations using snake-like progression
for run in number_of_datapoints:
    if counter < size_of_x:
        x = x + direction
    else:
        direction = direction * -1
        x = x + direction
        y = y + 1
    write_locations.writerow({'x': x, 'y': y})


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#-------------------------  IEEE signal signature  -------------------------
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Set the trigger for logging the data
log_data = 0 

# Log the data directly onto the .csv file
with open('IEEE_signal_database.csv') as database:
    sensors = ['IEEE_0', 'IEEE_1']
    write_database = csv.DictWriter(database, fieldnames = sensors)
    write_database.writeheader()

for run in number_of_datapoints:
    log_data = 0 # specify pin input in the future
    if log_data == 1:
        run = run + 1
        IEEE_0 = 0 # specify pin input in the future
        IEEE_1 = 0 # specify pin input in the future
        write_database.writerow({'IEEE_0': IEEE_0, 'IEEE_1': IEEE_1})