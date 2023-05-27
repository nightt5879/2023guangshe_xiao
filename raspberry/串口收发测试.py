import serial
com1 = serial.Serial("/dev/ttyAMA1",115200)
com2 = serial.Serial("/dev/ttyAMA2",115200)


com1.write(b'\x01')

print(com2.read())