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

otto = otto_communication_pb2.StatusMessage()
otto.linear_velocity = 0.3
otto.angular_velocity = 0.0
otto.delta_millis = 500
otto.status = otto_communication_pb2.StatusMessage.Status.RUNNING

encode_buffer = otto.SerializeToString()
print(encode_buffer)
print(len(encode_buffer))

while 1:
  ser.write(encode_buffer)
  ser.reset_output_buffer()
  print(encode_buffer)
  time.sleep(0.005)
