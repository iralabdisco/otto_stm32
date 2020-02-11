import time, serial, struct
import otto_communication_pb2
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

my_velocity = otto_communication_pb2.VelocityCommand()
my_velocity.linear_velocity = 0.3
my_velocity.angular_velocity = 0.0
while 1:
  print(ang_vel_cmd)
  print(lin_vel_cmd)
  ang_vel_cmd = ang_vel_cmd + 0.1
  lin_vel_cmd = lin_vel_cmd + 0.1
  msg_output_buffer = struct.pack('<ff', lin_vel_cmd, ang_vel_cmd)
  ser.write(msg_output_buffer)
  ser.reset_output_buffer()
