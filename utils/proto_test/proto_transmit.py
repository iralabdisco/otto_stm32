import velocities_pb2

import time, serial, struct
from serial import SerialException
ser = serial.Serial(
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        rtscts=False)
while (ser.is_open == False):
	try:
		ser.port = '/dev/ttyUSB0'
		ser.open()
	except SerialException:
		print("couldn't open ttyUSB0, check the connection")
		time.sleep(2)

print("port open")
message = velocities_pb2.Velocities()

while (1):
    message.linear_vel += 0.1
    message.angular_vel += 0.1
    buffer = message.SerializeToString()
    ser.write(buffer)
    print(message)
    print(buffer)
    ser.reset_output_buffer()
    time.sleep(3)