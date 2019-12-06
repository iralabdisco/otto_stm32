import serial, struct, time
ser = serial.Serial(
	port='/dev/ttyUSB0',
        baudrate=115200,
        parity=serial.PARITY_ODD,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        rtscts=False)
test2 = 0
while 1:
#	ser.reset_input_buffer()
#	buffer = ser.read(12)
#	msg_received = struct.unpack('<fff', buffer)
#	print(msg_received)
#	print(buffer)
	ang_vel_cmd = 1.2
	lin_vel_cmd = 7.5
	msg_output_buffer = struct.pack('<ff', ang_vel_cmd, lin_vel_cmd)
	test = ser.write(msg_output_buffer)
	print(test)
	test2 = test2 + test
	print(test2)
	ser.reset_output_buffer()
	time.sleep(1)
