import move
import time

c = move.Car()
c.car_stop()
print("init done")
# c.car_forward(400,400)
# time.sleep(2)
# c.car_turn_right_6050(39000)
for i in range(12):
    c.car_turn_right_6050(37000)
    time.sleep(2)
# c.car_turn_left(400,400)
# time.sleep(10)
# c.car_stop()
print("test done")
