import move
import time

c = move.Car()
DIS_X = 45
DIS_Y = 40
def move_forward(distance):
    c.car_send_distance_positive(distance, 0)
    c.car_cheak_data()

def move_backward(distance):
    c.car_send_distance_negative(distance, 0)
    c.car_cheak_data()

def move_left(distance):
    c.car_send_distance_negative(0, distance)
    c.car_cheak_data()

def move_right(distance):
    c.car_send_distance_positive(0, distance)
    c.car_cheak_data()
if __name__ == "__main__":
    # c.control_6050(0)
    # time.sleep(1)
    # c.control_6050(1)
    move_forward(DIS_Y)
    move_backward(DIS_Y)
    print("test done")
