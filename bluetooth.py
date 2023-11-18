# import serial
# #ser = serial.Serial('/dev/tty.HC-05RDC2023Group22', 115200)  # open serial port
# ser= serial.Serial('COM7')
# print(ser.name)         # check which port was really used

# ser.write(b'a\r\n')     # write a string
# ser.close()  


# import serial
# import time

# z1baudrate = 115200
# z1port = '/dev/tty.Bluetooth-Incoming-Port'  # set the correct port before run it

# z1serial = serial.Serial(port=z1port, baudrate=z1baudrate)
# z1serial.timeout = 2  # set read timeout
# # print z1serial  # debug serial.
# print(z1serial.is_open)  # True for opened
# if z1serial.is_open:
#     while True:
#         size = z1serial.inWaiting()
#         if size:
#             data = z1serial.read(size)
#             print(data)
#         else:
#             print('no data')

#         z1serial.write(b'a\r\n')
#         time.sleep(1)
        

# else:
#     print('z1serial not open')





# import serial

# port = "/dev/tty.ZhanibeksS22" 
# #port = '/dev/tty.Bluetooth-Incoming-Port'  # set the correct port before run it

# ser = serial.Serial(port, baudrate=115200, timeout=1) 


# while True:
#    data = ser.readline()
#    print(data)


# import serial
# import time

# ser = serial.Serial('/dev/tty.Bluetooth-Incoming-Port', 9600, timeout=1)

# while True:
#   # Read data
#   data = ser.readline()
#   print("fdas", data)
#   time.sleep(2)
#   # Write data
#   msg = "Hello from Python" 
#   ser.write((msg + "\r\n").encode('utf-8'))
#   time.sleep(2)

# ser.close()

import serial

# Replace '/dev/tty.YourDeviceName-SerialPortName' with your actual serial port
serial_port = '/dev/tty.ZhanibeksS22'
#serial_port = '/dev/tty.Bluetooth-Incoming-Port'
baud_rate = 9600  # Replace with the baud rate of your Bluetooth device

try:
    ser = serial.Serial(serial_port, baud_rate)
    while True:
        # Read data from the serial port
        data = ser.readline().decode().strip()
        print(data)  # Do something with the received data
        
except serial.SerialException as e:
    print("Serial port connection failed:", e)
finally:
    if ser.is_open:
        ser.close()
