import pigpio

# Use pigpio daemon to control general-purpose input/outputs more freely
pi = pigpio.pi()

# For TX of IEEE_0: (board 16, BCM 23)
# For TX of IEEE_1: (board 18, BCM 24)
IEEE_0_TX = 1
IEEE_1_TX = 1

# For RX of IEEE_0: (board 16, BCM 23)
# For RX of IEEE_1: (board 18, BCM 24)
IEEE_0_RX = 16
IEEE_1_RX = 18
baud = 9600

pi.set_mode(IEEE_0_TX, pigpio.OUTPUT)
pi.set_mode(IEEE_1_TX, pigpio.OUTPUT)

# CLose both RX pins if open
# Exceptions is turned off to prevent error in closing unopened GPIO
pigpio.exceptions = False
pi.bb_serial_read_close(IEEE_0_RX)
pi.bb_serial_read_close(IEEE_1_RX)
pigpio.exceptions = True

# Set up the commands for getting the signal strength from the transmitters
pi.wave_clear()
command = 'DB'
pi.wave_add_serial(IEEE_0_TX, baud, command)
get_strength_0 = pi.wave_create()
pi.wave_add_serial(IEEE_1_TX, baud, command)
get_strength_1 = pi.wave_create()

# Open pins for reading of serial data
pi.bb_serial_read_open(IEEE_0_RX, baud)
pi.bb_serial_read_open(IEEE_1_RX, baud)

# Send signal strength from the first transmitter
pi.wave_send_once(get_strength_0)

# Wait until all data has been sent
while pi.wave_tx_busy():
      pass

# Read signal strength data from the first transmitter
(temp_data_0, IEEE_0) = pi.bb_serial_read(IEEE_0_RX)

# Request signal strength from the second transmitter
pi.wave_send_once(get_strength_1)

# Wait until all data has been sent
while pi.wave_tx_busy():
      pass

# Read signal strength data from the second transmitter
(temp_data_1, IEEE_1) = pi.bb_serial_read(IEEE_1_RX)

# Convert bytearray to integer 
IEEE_0 = int.from_bytes(IEEE_0)  
IEEE_1 = int.from_bytes(IEEE_1)  

print("IEEE_0: " + str(IEEE_0) + "\n")
print("IEEE_1: " + str(IEEE_1) + "\n")
    
# Close pigpio
pi.wave_delete(0)
pi.wave_delete(1)
pi.bb_serial_read_close(IEEE_0_RX)
pi.bb_serial_read_close(IEEE_1_RX)
pi.stop()