import time, serial, struct
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
        ser.port = '/dev/ttyUSB1'
        ser.open()
    except SerialException:
        print("couldn't open ttyUSB1, check the connection")
        time.sleep(2)

print("port open")
ser.reset_input_buffer()
ser.rts = False
while 1:
    ser.reset_input_buffer()
    buffer = ser.read(4)
    msg_received = struct.unpack('i', buffer)
    print(msg_received)
    print(buffer)

