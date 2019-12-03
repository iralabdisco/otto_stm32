import serial, struct
ser = serial.Serial(
	port='/dev/ttyUSB0',
        baudrate=115200,
        parity=serial.PARITY_ODD,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.EIGHTBITS,
        rtscts=True)
while 1:
	buffer = ser.read(12)
	msg_received = struct.unpack('<fff', buffer)
	print(msg_received)
