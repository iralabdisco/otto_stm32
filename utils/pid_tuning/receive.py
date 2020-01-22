import serial, struct, time
ser = serial.Serial(
	port='/dev/ttyUSB0',
        baudrate=115200,
        parity=serial.PARITY_ODD,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        rtscts=False)

while 1:
	ser.reset_input_buffer()
	buffer = ser.read(8)
	msg_received = struct.unpack('<fi', buffer)
	print(msg_received)
	print(buffer)	

