import serial, struct, time
ser = serial.Serial(
	port='/dev/ttyUSB0',
        baudrate=115200,
        parity=serial.PARITY_ODD,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        rtscts=False)
ang_vel_cmd = 0
lin_vel_cmd = 1
while 1:
	msg_output_buffer = struct.pack('<ff', ang_vel_cmd, lin_vel_cmd)
	ser.write(msg_output_buffer)
	print(ang_vel_cmd)
	print(lin_vel_cmd)
#	ang_vel_cmd = ang_vel_cmd - 1
#	lin_vel_cmd = lin_vel_cmd - 1
	ser.reset_output_buffer()
	time.sleep(3)
