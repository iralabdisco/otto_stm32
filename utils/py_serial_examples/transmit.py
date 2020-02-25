import time, serial, struct
import otto_communication_pb2
from serial import SerialException

ser = serial.Serial(
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        rtscts=True,
        exclusive=None)
while (ser.is_open == False):
	try:
		ser.port = '/dev/ttyUSB0'
		ser.open()
	except SerialException:
		print("couldn't open ttyUSB0, check the connection")
		time.sleep(2)

print("open port")
ser.reset_output_buffer()
ciao = 5
while 1:
  msg_output_buffer = struct.pack('i', ciao)
  print (ser.cts)
  if (ser.cts == True):
    ser.write(msg_output_buffer)
    print(msg_output_buffer)
    ser.reset_output_buffer()
    ciao = ciao + 1
  time.sleep(5)