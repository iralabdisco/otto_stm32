import serial, struct, time, signal, sys
from datetime import datetime

ser = serial.Serial(
	port='/dev/ttyUSB0',
        baudrate=115200,
        parity=serial.PARITY_ODD,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        rtscts=False)

def signal_handler(sig, frame):
	print('Logging stopped')
	print('Info logged on ' + file_name)
	log_file.close()
	sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

user_input = input("Enter file name: ")

file_name = user_input + "-" +  datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + ".csv"

log_file = open("data/" + file_name, 'w')
log_file.write("DutyCycle, Velocity \n")

while 1:
	ser.reset_input_buffer()
	buffer = ser.read(8)
	msg_received = struct.unpack('<ff', buffer)
	print(msg_received[0])
	print(msg_received[1])
	log_file.write("%f %f \n" % (msg_received[0], msg_received[1]))	


