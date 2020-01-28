import serial, struct, time
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
print("yay")


pid_select = float(input("Enter 1 for left tuning, 2 for right tuning, 3 for cross tuning: "))
if (pid_select == 3):
	pid_lin_vel = float(input("Enter the linear velocity setpoint: "))
	pid_ang_vel = float(input("Enter the angular velocity setpoint: "))
	pid_fixed_setpoint = 0
else: 
	pid_fixed_setpoint = float(input("Enter the setpoint: "))
	pid_lin_vel = 0
	pid_ang_vel = 0
kp = float(input("Enter kp: "))
ki = float(input("Enter ki: "))
kd = float(input("Enter kd: "))
msg_output = struct.pack('<fffffff', pid_select, pid_fixed_setpoint, pid_lin_vel, pid_ang_vel, 
kp, ki, kd)
ser.write(msg_output)
ser.flush()
ser.close()

