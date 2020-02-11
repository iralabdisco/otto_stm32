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
encode_buffer = my_velocity.SerializeToString()
print(encode_buffer)
print(len(encode_buffer))

while 1:
  my_velocity.linear_velocity = my_velocity.linear_velocity + 0.01
  encode_buffer = my_velocity.SerializeToString()
  print(my_velocity)
  print(encode_buffer)
  ser.write(encode_buffer)
  ser.reset_output_buffer()
  time.sleep(0.5)
