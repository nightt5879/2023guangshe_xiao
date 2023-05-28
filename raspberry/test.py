import move
import time

#init
c = move.Car()
i = move.Infrared()
def turn_left_90(time_move):
    c.car_turn_left(500, 500)
    time.sleep(time_move)
    c.car_stop()

def turn_right_90(time_move):
    c.car_turn_right(500, 500)
    time.sleep(time_move)
    c.car_stop()


def car_move_test():
    c.car_forward(1000,1000)
    time.sleep(5)
    # for i in range(6):
    #     turn_left_90(0.5)
    #     time.sleep(0.5)
    #     turn_right_90(0.5)
    #     time.sleep(0.5)
    c.car_back(1000,1000)
    time.sleep(5)
    c.car_stop()

if __name__ == '__main__':
    print(i.infrared_read())
    print("test done")

