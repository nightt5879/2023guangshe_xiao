import move
import time

c = move.Car()
c.car_stop()
print("init done")
# c.car_forward(400,400)
# time.sleep(2)
# c.car_forward_6050()
# time.sleep(5)
c.car_stop()
c.car_turn_right_6050(12000)
# c.car_turn_right_6050(10000)

print("test done")

"""
if（第一位等于 0x05 && 第二位等于）
{
    左转的GPIO
    while(1)
    {
        6050
        if （大于一定值）
        {
            break;
        }
    }
}
else if （第一位等于 0x05 && 第二位等于）
    {
        右转的GPIO
    while(1)
    {
        6050
        if （大于一定值）
        {
            break;
        }
    }
    }

"""
