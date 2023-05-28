import serial
import time
com1 = serial.Serial("/dev/ttyAMA2",115200)
com2 = serial.Serial("/dev/ttyAMA2",115200)

a = 0x01

while True:
    data = [0xFF, a, 0x02, 0x03, 0x04, 0xFE]
    ##sssdfsdfsqqq
    com1.write(data)
    time.sleep(0.1)
    a += 1
    print("send done")

print(com2.read())