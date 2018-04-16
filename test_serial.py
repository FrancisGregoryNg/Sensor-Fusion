import pigpio

# Use pigpio daemon to control general-purpose input/outputs more freely
pi = pigpio.pi()

# Open pins for bit bang reading of serial data
# For IEEE_0: (board 16, BCM 23)
# For IEEE_1: (board 18, BCM 24)
pi.bb_serial_read_open(14, 9600)
pi.bb_serial_read_open(15, 9600)

(IEEE_0_start, temp_data_0) = pi.bb_serial_read(23)
(IEEE_1_start, temp_data_1) = pi.bb_serial_read(24) 

for run in range(0, 5):
    print("IEEE_0_start: " + str(IEEE_0_start) + "\n")
    print("temp_data_0: " + str(temp_data_0) + "\n")
    print("IEEE_1_start: " + str(IEEE_1_start) + "\n")
    print("temp_data_1: " + str(temp_data_1) + "\n")
    
pi.bb_serial_read_close(14)
pi.bb_serial_read_close(15)