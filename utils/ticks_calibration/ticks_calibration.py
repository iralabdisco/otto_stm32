import serial, struct, time
ser = serial.Serial(
	port='/dev/ttyUSB0',
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        rtscts=False)

while 1:
	linear_velocity = float(input("Enter linear velocity setpoint: "))
	angular_velocity = float(input("Enter angular velocity setpoint: "))
	msg_output = struct.pack('<ff', linear_velocity, angular_velocity)
	ser.write(msg_output)
	ser.flush()

ser.close()

