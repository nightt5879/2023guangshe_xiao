import move
import time

c = move.Car()
c.car_stop()
print("init done")
# c.car_forward(400,400)
# time.sleep(2)
c.car_forward_6050()
time.sleep(5)
c.car_stop()
# c.car_turn_right_6050(3700)
# c.car_turn_right_6050(10000)

print("test done")
