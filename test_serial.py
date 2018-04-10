import pigpio

# Use pigpio daemon to control general-purpose input/outputs more freely
pi = pigpio.pi()

# Open pins for bit bang reading of serial data
# For IEEE_0: (board 8, BCM 14)
# For IEEE_1: (board 10, BCM 15)
pi.bb_serial_read_open(14, 115200)
pi.bb_serial_read_open(15, 115200) 

(IEEE_0_start, temp_data_0) = pi.bb_serial_read()
(IEEE_1_start, temp_data_1) = pi.bb_serial_read() 