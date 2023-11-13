import serial
ser = serial.Serial('/dev/tty.HC-05RDC2023Group22', 115200)  # open serial port
print(ser.name)         # check which port was really used

ser.write(b'a')     # write a string
ser.close()  

























