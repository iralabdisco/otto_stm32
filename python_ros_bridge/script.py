import serial, struct
ser = serial.Serial('/dev/ttyUSB0')
buffer = ser.read(12)
angular_vel = struct.unpack('f', buffer[0:4])
linear_vel = struct.unpack('f', buffer[4:8])
delta_time = struct.unpack('f', buffer[8:12])
print(angular_vel)
print(linear_vel)
print(delta_time)
