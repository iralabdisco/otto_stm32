import time, serial, struct
from serial import SerialException
ser = serial.Serial(
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        rtscts=False,
        exclusive=None)
while (ser.is_open == False):
	try:
		ser.port = '/dev/ttyUSB0'
		ser.open()
	except SerialException:
		print("couldn't open ttyUSB0, check the connection")
		time.sleep(2)

print("port open")

while 1:
	ser.reset_input_buffer()
	buffer = ser.read(8)
	msg_received = struct.unpack('<ff', buffer)
	print(msg_received)
	print(buffer)

