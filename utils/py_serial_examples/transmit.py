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

print("open port")
ang_vel_cmd = 0
lin_vel_cmd = 0.5
while 1:
	msg_output_buffer = struct.pack('<ff', lin_vel_cmd, ang_vel_cmd)
	ser.write(msg_output_buffer)
	print(ang_vel_cmd)
	print(lin_vel_cmd)
	ser.reset_output_buffer()
	time.sleep(3)
