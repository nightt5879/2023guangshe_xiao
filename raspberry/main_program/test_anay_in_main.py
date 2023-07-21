import move
import time

c = move.Car()

if __name__ == "__main__":
    # print("test start")
    # c.car_send_distance_positive(10,0)
    # c.car_com1.write(b"1")
    # print("send done")
    c.car_send_distance_positive(40, 0)
    c.car_cheak_data()
    time.sleep(0.1)
    c.car_send_distance_negative(0,50)
    c.car_cheak_data()
    time.sleep(0.1)
    c.car_send_distance_negative(37,0)
    c.car_cheak_data()
    time.sleep(0.1)
    c.car_send_distance_negative(0,50)
    c.car_cheak_data()
    time.sleep(0.1)
    c.car_send_distance_positive(40,0)
    c.car_cheak_data()
    time.sleep(0.1)
    c.car_send_distance_negative(0,50)
    c.car_cheak_data()
    # print("receive done")
    # while True:
    #     c.car_cheak_data()
    #     print("1")

"""
Traceback (most recent call last):
  File "/usr/lib/python3/dist-packages/serial/serialposix.py", line 265, in open
    self.fd = os.open(self.portstr, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
FileNotFoundError: [Errno 2] No such file or directory: '/dev/ttyAMA2'

During handling of the above exception, another exception occurred:
"""