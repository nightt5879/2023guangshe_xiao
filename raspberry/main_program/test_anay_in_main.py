import move

c = move.Car()

if __name__ == "__main__":
    print("test start")
    c.car_forward_mecanum(10,2,0)
    # c.car_com1.write(b"1")
    print("send done")
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