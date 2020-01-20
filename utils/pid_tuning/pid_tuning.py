import serial, struct, time
ser = serial.Serial(
	port='/dev/ttyUSB0',
        baudrate=115200,
        parity=serial.PARITY_ODD,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        rtscts=False)
pid_select = float(input("Enter 0 for left tuning, 1 for right tuning, 2 for cross tuning: "))
pid_setpoint = float(input("Enter the setpoint: "))
kp = float(input("Enter kp: "))
ki = float(input("Enter ki: "))
kd = float(input("Enter kd: "))
msg_output = struct.pack('<fffff', pid_select, pid_setpoint, kp, ki, kd)
ser.write(msg_output)
ser.reset_output_buffer()
ser.close()

