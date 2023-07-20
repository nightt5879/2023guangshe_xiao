import serial
import RPi.GPIO as GPIO

class Car:
    def __init__(self):
        """
        init the car
        """
        self.car_com1 = serial.Serial("/dev/ttyAMA1", 115200)  # init the car com
        self.data = [0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE]  # init the data
        """
        there are eight digits in the data, first 0xFF is the heda, 0xFE is the end
        the second and third digits are the operation code,
        the fourth and fifth digits are the one wheel high and low eight bits of speed
        the sixth and seventh digits are the other wheel high and low eight bits of speed
        operation:
        0x00,0x00:stop
        0x01,0x00:forward
        0x02,0x00:back
        0x03,0x00:turn left
        0x04,0x00:turn right
        0x05,0x11:turn left with mpu6050 (the fourth and fifth digits are the angle)
        0X05,0X21:turn right with mpu6050 (the fourth and fifth digits are the angle)
        0x06,0x00:let the car go forward with the mpu6050s
        """

    def car_stop(self):
        """
        the car stop
        """
        for i in range(6):
            self.data[i + 1] = 0x00  # clear the data
        self.car_com1.write(self.data)  # send the data

    def car_forward(self, left_speed, right_speed):
        """
        the car forward
        :param left_speed: the left wheel speed(range 0~1000)
        :param right_speed: the right wheel speed(range 0~1000)
        """
        self.data[1] = 0x01
        self.data[2] = 0x00
        self.data[3] = left_speed >> 8
        self.data[4] = left_speed & 0xFF
        self.data[5] = right_speed >> 8
        self.data[6] = right_speed & 0xFF
        self.car_com1.write(self.data)

    def car_back(self, left_speed, right_speed):
        """
        the car back
        :param left_speed: the left wheel speed(range 0~1000)
        :param right_speed: the right wheel speed(range 0~1000)
        """
        self.data[1] = 0x02
        self.data[2] = 0x00
        self.data[3] = left_speed >> 8
        self.data[4] = left_speed & 0xFF
        self.data[5] = right_speed >> 8
        self.data[6] = right_speed & 0xFF
        self.car_com1.write(self.data)

    def car_turn_left(self, left_speed, right_speed):
        """
        the car turn left
        :param left_speed: the left wheel speed(range 0~1000)
        :param right_speed: the right wheel speed(range 0~1000)
        """
        self.data[1] = 0x03
        self.data[2] = 0x00
        self.data[3] = left_speed >> 8
        self.data[4] = left_speed & 0xFF
        self.data[5] = right_speed >> 8
        self.data[6] = right_speed & 0xFF
        self.car_com1.write(self.data)

    def car_turn_right(self, left_speed, right_speed):
        """
        the car turn right
        :param left_speed: the left wheel speed(range 0~1000)
        :param right_speed: the right wheel speed(range 0~1000)
        """
        self.data[1] = 0x04
        self.data[2] = 0x00
        self.data[3] = left_speed >> 8
        self.data[4] = left_speed & 0xFF
        self.data[5] = right_speed >> 8
        self.data[6] = right_speed & 0xFF
        self.car_com1.write(self.data)

    def car_turn_left_6050(self,turn_angel):
        """
        the car turn left with mpu6050
        :param turn_angel: the turn angel
        """
        self.data[1] = 0x05
        self.data[2] = 0x11
        self.data[3] = turn_angel >> 8
        self.data[4] = turn_angel & 0xFF
        self.data[5] = 0x00
        self.data[6] = 0x00
        self.car_com1.write(self.data)

    def car_turn_right_6050(self,turn_angel):
        """
        the car turn right with mpu6050
        :param turn_angel: the turn angel
        """
        self.data[1] = 0x05
        self.data[2] = 0x21
        self.data[3] = turn_angel >> 8
        self.data[4] = turn_angel & 0xFF
        self.data[5] = 0x00
        self.data[6] = 0x00
        self.car_com1.write(self.data)

    def car_forward_6050(self):
        """
        the car go forward with mpu6050
        """
        self.data[1] = 0x06
        self.data[2] = 0x00
        self.car_com1.write(self.data)

    def car_cheak_data(self):
        """
        the car cheak data
        :param data_byte: the data byte
        """
        received_data = self.car_com1.read()
        print(received_data)

    def car_forward_mecanum(self, forward_speed, correction_speed, distance):
        """
        the car forward with mecanum wheel
        :param forward_speed: the speed of forward
        :param correction_speed: the speed of correction
        :parm distance: the distance of move, if it's 0 it mean move until the corner
        """
        self.data[1] = 0x07
        self.data[2] = distance
        self.data[3] = forward_speed
        self.data[4] = correction_speed
        self.car_com1.write(self.data)

    def car_backward_mecaum(self, backward_speed, correction_speed, distance):
        """
        the car backward with mecanum wheel
        :param backward_speed: the speed of backward
        :param correction_speed: the speed of correction
        :parm distance: the distance of move, if it's 0 it mean move until the corner
        """
        self.data[1] = 0x08
        self.data[2] = distance
        self.data[3] = backward_speed
        self.data[4] = correction_speed
        self.car_com1.write(self.data)

    def car_move_left_mecaum(self, move_left_speed, correction_speed, distance):
        """
        the car moveleft with mecanum wheel
        :param moveleft_speed: the speed of moveleft
        :param correction_speed: the speed of correction
        :parm distance: the distance of move, if it's 0 it mean move until the corner
        """
        self.data[1] = 0x09
        self.data[2] = distance
        self.data[3] = move_left_speed
        self.data[4] = correction_speed
        self.car_com1.write(self.data)

    def car_move_right_mecaum(self, move_right_speed, correction_speed, distance):
        """
        the car moveright with mecanum wheel
        :param moveright_speed: the speed of moveright
        :param correction_speed: the speed of correction
        :parm distance: the distance of move, if it's 0 it mean move until the corner
        """
        self.data[1] = 0x0A
        self.data[1] = 0x07
        self.data[2] = distance
        self.data[3] = move_right_speed
        self.data[4] = correction_speed
        self.car_com1.write(self.data)
class Infrared:
    def __init__(self):
        """
        init the infrared
        """
        GPIO.setmode(GPIO.BCM)  #set the GPIO mode
        #the infrareds are from left to right, i.e. left outside left inside right inside right outside
        GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def infrared_read(self):
        """
        the infrared read
        :return the infrared value: it is a list, the first element is the left outside infrared value,
        """
        infrared_value = [0, 0, 0, 0]
        infrared_value[0] = GPIO.input(27)
        infrared_value[1] = GPIO.input(21)
        infrared_value[2] = GPIO.input(17)
        infrared_value[3] = GPIO.input(22)
        return infrared_value

path = [[{'move_mode': '前进', 'now_distance': (1, 0, 0, 0), 'target_distance': (0, 0, 1, 1), 'now_xy': (19, 1),
      'target_xy': (19, 3)},
     {'move_mode': '左转', 'now_distance': (0, 0, 1, 1), 'target_distance': (1, 0, 0, 1), 'now_xy': (19, 3),
      'target_xy': (19, 3)},
     {'move_mode': '前进', 'now_distance': (1, 0, 0, 1), 'target_distance': (0, 0, 1, 1), 'now_xy': (19, 3),
      'target_xy': (17, 3)},
     {'move_mode': '左转', 'now_distance': (0, 0, 1, 1), 'target_distance': (1, 0, 0, 1), 'now_xy': (17, 3),
      'target_xy': (17, 3)},
     {'move_mode': '前进', 'now_distance': (1, 0, 0, 1), 'target_distance': (0, 1, 1, 0), 'now_xy': (17, 3),
      'target_xy': (17, 1)},
     {'move_mode': '右转', 'now_distance': (0, 1, 1, 0), 'target_distance': (1, 1, 0, 0), 'now_xy': (17, 1),
      'target_xy': (17, 1)},
     {'move_mode': '前进', 'now_distance': (1, 1, 0, 0), 'target_distance': (0, 1, 1, 0), 'now_xy': (17, 1),
      'target_xy': (15, 1)},
     {'move_mode': '右转', 'now_distance': (0, 1, 1, 0), 'target_distance': (1, 1, 0, 0), 'now_xy': (15, 1),
      'target_xy': (15, 1)},
     {'move_mode': '前进', 'now_distance': (1, 1, 0, 0), 'target_distance': (0, 0, 1, 1), 'now_xy': (15, 1),
      'target_xy': (15, 3)},
     {'move_mode': '左转', 'now_distance': (0, 0, 1, 1), 'target_distance': (1, 0, 0, 1), 'now_xy': (15, 3),
      'target_xy': (15, 3)},
     {'move_mode': '前进', 'now_distance': (1, 0, 0, 1), 'target_distance': (0, 2, 1, 0), 'now_xy': (15, 3),
      'target_xy': (13, 3)},
     {'move_mode': '右转', 'now_distance': (0, 2, 1, 0), 'target_distance': (2, 1, 0, 0), 'now_xy': (13, 3),
      'target_xy': (13, 3)},
     {'move_mode': '前进', 'now_distance': (2, 1, 0, 0), 'target_distance': (1, 0, 1, 1), 'now_xy': (13, 3),
      'target_xy': (13, 5)},
     {'move_mode': '前进', 'now_distance': (1, 0, 1, 1), 'target_distance': (0, 2, 2, 0), 'now_xy': (13, 5),
      'target_xy': (13, 7)},
     {'move_mode': '右转', 'now_distance': (0, 2, 2, 0), 'target_distance': (2, 2, 0, 0), 'now_xy': (13, 7),
      'target_xy': (13, 7)},
     {'move_mode': '前进', 'now_distance': (2, 2, 0, 0), 'target_distance': (1, 0, 1, 1), 'now_xy': (13, 7),
      'target_xy': (15, 7)},
     {'move_mode': '前进', 'now_distance': (1, 0, 1, 1), 'target_distance': (0, 0, 2, 2), 'now_xy': (15, 7),
      'target_xy': (17, 7)},
     {'move_mode': '左转', 'now_distance': (0, 0, 2, 2), 'target_distance': (2, 0, 0, 2), 'now_xy': (17, 7),
      'target_xy': (17, 7)},
     {'move_mode': '前进', 'now_distance': (2, 0, 0, 2), 'target_distance': (1, 1, 1, 0), 'now_xy': (17, 7),
      'target_xy': (17, 9)},
     {'move_mode': '右转', 'now_distance': (1, 1, 1, 0), 'target_distance': (1, 1, 0, 1), 'now_xy': (17, 9),
      'target_xy': (17, 9)},
     {'move_mode': '前进', 'now_distance': (1, 1, 0, 1), 'target_distance': (0, 2, 1, 0), 'now_xy': (17, 9),
      'target_xy': (19, 9)},
     {'move_mode': '右转', 'now_distance': (0, 2, 1, 0), 'target_distance': (2, 1, 0, 0), 'now_xy': (19, 9),
      'target_xy': (19, 9)},
     {'move_mode': '前进', 'now_distance': (2, 1, 0, 0), 'target_distance': (1, 0, 1, 0), 'now_xy': (19, 9),
      'target_xy': (19, 7)},
     {'move_mode': '前进', 'now_distance': (1, 0, 1, 0), 'target_distance': (0, 1, 2, 0), 'now_xy': (19, 7),
      'target_xy': (19, 5)},
     {'move_mode': '右转', 'now_distance': (0, 1, 2, 0), 'target_distance': (1, 2, 0, 0), 'now_xy': (19, 5),
      'target_xy': (19, 5)},
     {'move_mode': '前进', 'now_distance': (1, 2, 0, 0), 'target_distance': (0, 0, 1, 0), 'now_xy': (19, 5),
      'target_xy': (17, 5)}], [
        {'move_mode': '左转', 'now_distance': (0, 0, 1, 0), 'target_distance': (0, 0, 0, 1), 'now_xy': (17, 5),
         'target_xy': (17, 5)},
        {'move_mode': '左转', 'now_distance': (0, 0, 0, 1), 'target_distance': (1, 0, 0, 0), 'now_xy': (17, 5),
         'target_xy': (17, 5)},
        {'move_mode': '前进', 'now_distance': (1, 0, 0, 0), 'target_distance': (0, 0, 1, 2), 'now_xy': (17, 5),
         'target_xy': (19, 5)},
        {'move_mode': '左转', 'now_distance': (0, 0, 1, 2), 'target_distance': (2, 0, 0, 1), 'now_xy': (19, 5),
         'target_xy': (19, 5)},
        {'move_mode': '前进', 'now_distance': (2, 0, 0, 1), 'target_distance': (1, 0, 1, 0), 'now_xy': (19, 5),
         'target_xy': (19, 7)},
        {'move_mode': '前进', 'now_distance': (1, 0, 1, 0), 'target_distance': (0, 0, 2, 1), 'now_xy': (19, 7),
         'target_xy': (19, 9)},
        {'move_mode': '左转', 'now_distance': (0, 0, 2, 1), 'target_distance': (1, 0, 0, 2), 'now_xy': (19, 9),
         'target_xy': (19, 9)},
        {'move_mode': '前进', 'now_distance': (1, 0, 0, 2), 'target_distance': (0, 1, 1, 1), 'now_xy': (19, 9),
         'target_xy': (17, 9)},
        {'move_mode': '右转', 'now_distance': (0, 1, 1, 1), 'target_distance': (1, 1, 1, 0), 'now_xy': (17, 9),
         'target_xy': (17, 9)},
        {'move_mode': '前进', 'now_distance': (1, 1, 1, 0), 'target_distance': (0, 1, 2, 0), 'now_xy': (17, 9),
         'target_xy': (17, 11)},
        {'now_distance': (0, 1, 2, 0), 'target_distance': (0, 0, 1, 2), 'now_xy': (17, 11),
         'target_xy': (17, 11), 'move_mode': '左转'}], [
        {'move_mode': '左转', 'now_distance': (0, 0, 1, 2), 'target_distance': (2, 0, 0, 1), 'now_xy': (17, 11),
         'target_xy': (17, 11)},
        {'move_mode': '左转', 'now_distance': (2, 0, 0, 1), 'target_distance': (1, 2, 0, 0), 'now_xy': (17, 11),
         'target_xy': (17, 11)},
        {'move_mode': '前进', 'now_distance': (1, 2, 0, 0), 'target_distance': (0, 0, 1, 4), 'now_xy': (17, 11),
         'target_xy': (19, 11)},
        {'move_mode': '左转', 'now_distance': (0, 0, 1, 4), 'target_distance': (4, 0, 0, 1), 'now_xy': (19, 11),
         'target_xy': (19, 11)},
        {'move_mode': '前进', 'now_distance': (4, 0, 0, 1), 'target_distance': (3, 0, 1, 1), 'now_xy': (19, 11),
         'target_xy': (19, 13)},
        {'move_mode': '前进', 'now_distance': (3, 0, 1, 1), 'target_distance': (2, 0, 2, 0), 'now_xy': (19, 13),
         'target_xy': (19, 15)},
        {'move_mode': '前进', 'now_distance': (2, 0, 2, 0), 'target_distance': (1, 0, 3, 0), 'now_xy': (19, 15),
         'target_xy': (19, 17)},
        {'now_distance': (1, 0, 3, 0), 'target_distance': (0, 1, 0, 3), 'now_xy': (19, 17),
         'target_xy': (19, 17), 'move_mode': '左转'}], [
        {'move_mode': '左转', 'now_distance': (0, 1, 0, 3), 'target_distance': (3, 0, 1, 0), 'now_xy': (19, 17),
         'target_xy': (19, 17)},
        {'move_mode': '前进', 'now_distance': (3, 0, 1, 0), 'target_distance': (2, 0, 2, 0), 'now_xy': (19, 17),
         'target_xy': (19, 15)},
        {'move_mode': '前进', 'now_distance': (2, 0, 2, 0), 'target_distance': (1, 1, 3, 0), 'now_xy': (19, 15),
         'target_xy': (19, 13)},
        {'move_mode': '右转', 'now_distance': (1, 1, 3, 0), 'target_distance': (1, 3, 0, 1), 'now_xy': (19, 13),
         'target_xy': (19, 13)},
        {'move_mode': '前进', 'now_distance': (1, 3, 0, 1), 'target_distance': (0, 1, 1, 0), 'now_xy': (19, 13),
         'target_xy': (17, 13)},
        {'move_mode': '右转', 'now_distance': (0, 1, 1, 0), 'target_distance': (1, 1, 0, 0), 'now_xy': (17, 13),
         'target_xy': (17, 13)},
        {'move_mode': '前进', 'now_distance': (1, 1, 0, 0), 'target_distance': (0, 0, 1, 3), 'now_xy': (17, 13),
         'target_xy': (17, 15)},
        {'move_mode': '左转', 'now_distance': (0, 0, 1, 3), 'target_distance': (3, 0, 0, 1), 'now_xy': (17, 15),
         'target_xy': (17, 15)},
        {'move_mode': '前进', 'now_distance': (3, 0, 0, 1), 'target_distance': (2, 1, 1, 1), 'now_xy': (17, 15),
         'target_xy': (15, 15)},
        {'move_mode': '左转', 'now_distance': (2, 1, 1, 1), 'target_distance': (1, 2, 1, 1), 'now_xy': (15, 15),
         'target_xy': (15, 15)},
        {'move_mode': '前进', 'now_distance': (1, 2, 1, 1), 'target_distance': (0, 3, 2, 0), 'now_xy': (15, 15),
         'target_xy': (15, 13)},
        {'move_mode': '右转', 'now_distance': (0, 3, 2, 0), 'target_distance': (3, 2, 0, 0), 'now_xy': (15, 13),
         'target_xy': (15, 13)},
        {'move_mode': '前进', 'now_distance': (3, 2, 0, 0), 'target_distance': (2, 0, 1, 1), 'now_xy': (15, 13),
         'target_xy': (13, 13)},
        {'move_mode': '左转', 'now_distance': (2, 0, 1, 1), 'target_distance': (1, 2, 0, 1), 'now_xy': (13, 13),
         'target_xy': (13, 13)},
        {'move_mode': '前进', 'now_distance': (1, 2, 0, 1), 'target_distance': (0, 0, 1, 0), 'now_xy': (13, 13),
         'target_xy': (13, 11)}], [
        {'move_mode': '左转', 'now_distance': (0, 0, 1, 0), 'target_distance': (0, 0, 0, 1), 'now_xy': (13, 11),
         'target_xy': (13, 11)},
        {'move_mode': '左转', 'now_distance': (0, 0, 0, 1), 'target_distance': (1, 0, 0, 0), 'now_xy': (13, 11),
         'target_xy': (13, 11)},
        {'move_mode': '前进', 'now_distance': (1, 0, 0, 0), 'target_distance': (0, 1, 1, 2), 'now_xy': (13, 11),
         'target_xy': (13, 13)},
        {'move_mode': '左转', 'now_distance': (0, 1, 1, 2), 'target_distance': (2, 0, 1, 1), 'now_xy': (13, 13),
         'target_xy': (13, 13)},
        {'move_mode': '前进', 'now_distance': (2, 0, 1, 1), 'target_distance': (1, 0, 2, 6), 'now_xy': (13, 13),
         'target_xy': (11, 13)},
        {'move_mode': '左转', 'now_distance': (1, 0, 2, 6), 'target_distance': (6, 1, 0, 2), 'now_xy': (11, 13),
         'target_xy': (11, 13)},
        {'move_mode': '前进', 'now_distance': (6, 1, 0, 2), 'target_distance': (5, 0, 1, 0), 'now_xy': (11, 13),
         'target_xy': (11, 11)},
        {'move_mode': '前进', 'now_distance': (5, 0, 1, 0), 'target_distance': (4, 0, 2, 0), 'now_xy': (11, 11),
         'target_xy': (11, 9)},
        {'move_mode': '前进', 'now_distance': (4, 0, 2, 0), 'target_distance': (3, 3, 3, 0), 'now_xy': (11, 9),
         'target_xy': (11, 7)},
        {'move_mode': '右转', 'now_distance': (3, 3, 3, 0), 'target_distance': (3, 3, 0, 3), 'now_xy': (11, 7),
         'target_xy': (11, 7)},
        {'move_mode': '前进', 'now_distance': (3, 3, 0, 3), 'target_distance': (2, 6, 1, 0), 'now_xy': (11, 7),
         'target_xy': (9, 7)},
        {'move_mode': '前进', 'now_distance': (2, 6, 1, 0), 'target_distance': (1, 1, 2, 0), 'now_xy': (9, 7),
         'target_xy': (7, 7)},
        {'move_mode': '右转', 'now_distance': (1, 1, 2, 0), 'target_distance': (1, 2, 0, 1), 'now_xy': (7, 7),
         'target_xy': (7, 7)},
        {'move_mode': '前进', 'now_distance': (1, 2, 0, 1), 'target_distance': (0, 0, 1, 0), 'now_xy': (7, 7),
         'target_xy': (7, 9)}], [
        {'move_mode': '左转', 'now_distance': (0, 0, 1, 0), 'target_distance': (0, 0, 0, 1), 'now_xy': (7, 9),
         'target_xy': (7, 9)},
        {'move_mode': '左转', 'now_distance': (0, 0, 0, 1), 'target_distance': (1, 0, 0, 0), 'now_xy': (7, 9),
         'target_xy': (7, 9)},
        {'move_mode': '前进', 'now_distance': (1, 0, 0, 0), 'target_distance': (0, 1, 1, 2), 'now_xy': (7, 9),
         'target_xy': (7, 7)},
        {'move_mode': '右转', 'now_distance': (0, 1, 1, 2), 'target_distance': (1, 1, 2, 0), 'now_xy': (7, 7),
         'target_xy': (7, 7)},
        {'move_mode': '前进', 'now_distance': (1, 1, 2, 0), 'target_distance': (0, 0, 3, 2), 'now_xy': (7, 7),
         'target_xy': (5, 7)},
        {'move_mode': '左转', 'now_distance': (0, 0, 3, 2), 'target_distance': (2, 0, 0, 3), 'now_xy': (5, 7),
         'target_xy': (5, 7)},
        {'move_mode': '前进', 'now_distance': (2, 0, 0, 3), 'target_distance': (1, 1, 1, 2), 'now_xy': (5, 7),
         'target_xy': (5, 5)},
        {'move_mode': '右转', 'now_distance': (1, 1, 1, 2), 'target_distance': (1, 1, 2, 1), 'now_xy': (5, 5),
         'target_xy': (5, 5)},
        {'move_mode': '前进', 'now_distance': (1, 1, 2, 1), 'target_distance': (0, 1, 3, 0), 'now_xy': (5, 5),
         'target_xy': (3, 5)},
        {'move_mode': '右转', 'now_distance': (0, 1, 3, 0), 'target_distance': (1, 3, 0, 0), 'now_xy': (3, 5),
         'target_xy': (3, 5)},
        {'move_mode': '前进', 'now_distance': (1, 3, 0, 0), 'target_distance': (0, 0, 1, 1), 'now_xy': (3, 5),
         'target_xy': (3, 7)},
        {'move_mode': '左转', 'now_distance': (0, 0, 1, 1), 'target_distance': (1, 0, 0, 1), 'now_xy': (3, 7),
         'target_xy': (3, 7)},
        {'move_mode': '前进', 'now_distance': (1, 0, 0, 1), 'target_distance': (0, 1, 1, 3), 'now_xy': (3, 7),
         'target_xy': (1, 7)},
        {'move_mode': '左转', 'now_distance': (0, 1, 1, 3), 'target_distance': (3, 0, 1, 1), 'now_xy': (1, 7),
         'target_xy': (1, 7)},
        {'move_mode': '前进', 'now_distance': (3, 0, 1, 1), 'target_distance': (2, 0, 2, 0), 'now_xy': (1, 7),
         'target_xy': (1, 5)},
        {'move_mode': '前进', 'now_distance': (2, 0, 2, 0), 'target_distance': (1, 0, 3, 0), 'now_xy': (1, 5),
         'target_xy': (1, 3)},
        {'now_distance': (1, 0, 3, 0), 'target_distance': (0, 1, 0, 3), 'now_xy': (1, 3), 'target_xy': (1, 3),
         'move_mode': '左转'}], [
        {'move_mode': '左转', 'now_distance': (0, 1, 0, 3), 'target_distance': (3, 0, 1, 0), 'now_xy': (1, 3),
         'target_xy': (1, 3)},
        {'move_mode': '前进', 'now_distance': (3, 0, 1, 0), 'target_distance': (2, 0, 2, 0), 'now_xy': (1, 3),
         'target_xy': (1, 5)},
        {'move_mode': '前进', 'now_distance': (2, 0, 2, 0), 'target_distance': (1, 1, 3, 0), 'now_xy': (1, 5),
         'target_xy': (1, 7)},
        {'move_mode': '前进', 'now_distance': (1, 1, 3, 0), 'target_distance': (0, 1, 4, 0), 'now_xy': (1, 7),
         'target_xy': (1, 9)},
        {'move_mode': '右转', 'now_distance': (0, 1, 4, 0), 'target_distance': (1, 4, 0, 0), 'now_xy': (1, 9),
         'target_xy': (1, 9)},
        {'move_mode': '前进', 'now_distance': (1, 4, 0, 0), 'target_distance': (0, 0, 1, 2), 'now_xy': (1, 9),
         'target_xy': (3, 9)}], [
        {'move_mode': '左转', 'now_distance': (0, 0, 1, 2), 'target_distance': (2, 0, 0, 1), 'now_xy': (3, 9),
         'target_xy': (3, 9)},
        {'move_mode': '前进', 'now_distance': (2, 0, 0, 1), 'target_distance': (1, 0, 1, 1), 'now_xy': (3, 9),
         'target_xy': (3, 11)},
        {'move_mode': '左转', 'now_distance': (1, 0, 1, 1), 'target_distance': (1, 1, 0, 1), 'now_xy': (3, 11),
         'target_xy': (3, 11)},
        {'move_mode': '前进', 'now_distance': (1, 1, 0, 1), 'target_distance': (0, 2, 1, 0), 'now_xy': (3, 11),
         'target_xy': (1, 11)},
        {'move_mode': '右转', 'now_distance': (0, 2, 1, 0), 'target_distance': (2, 1, 0, 0), 'now_xy': (1, 11),
         'target_xy': (1, 11)},
        {'move_mode': '前进', 'now_distance': (2, 1, 0, 0), 'target_distance': (1, 0, 1, 0), 'now_xy': (1, 11),
         'target_xy': (1, 13)},
        {'move_mode': '前进', 'now_distance': (1, 0, 1, 0), 'target_distance': (0, 1, 2, 0), 'now_xy': (1, 13),
         'target_xy': (1, 15)},
        {'move_mode': '右转', 'now_distance': (0, 1, 2, 0), 'target_distance': (1, 2, 0, 0), 'now_xy': (1, 15),
         'target_xy': (1, 15)},
        {'move_mode': '前进', 'now_distance': (1, 2, 0, 0), 'target_distance': (0, 0, 1, 0), 'now_xy': (1, 15),
         'target_xy': (3, 15)}], [
        {'move_mode': '左转', 'now_distance': (0, 0, 1, 0), 'target_distance': (0, 0, 0, 1), 'now_xy': (3, 15),
         'target_xy': (3, 15)},
        {'move_mode': '左转', 'now_distance': (0, 0, 0, 1), 'target_distance': (1, 0, 0, 0), 'now_xy': (3, 15),
         'target_xy': (3, 15)},
        {'move_mode': '前进', 'now_distance': (1, 0, 0, 0), 'target_distance': (0, 0, 1, 2), 'now_xy': (3, 15),
         'target_xy': (1, 15)},
        {'move_mode': '左转', 'now_distance': (0, 0, 1, 2), 'target_distance': (2, 0, 0, 1), 'now_xy': (1, 15),
         'target_xy': (1, 15)},
        {'move_mode': '前进', 'now_distance': (2, 0, 0, 1), 'target_distance': (1, 0, 1, 0), 'now_xy': (1, 15),
         'target_xy': (1, 13)},
        {'move_mode': '前进', 'now_distance': (1, 0, 1, 0), 'target_distance': (0, 0, 2, 1), 'now_xy': (1, 13),
         'target_xy': (1, 11)},
        {'move_mode': '左转', 'now_distance': (0, 0, 2, 1), 'target_distance': (1, 0, 0, 2), 'now_xy': (1, 11),
         'target_xy': (1, 11)},
        {'move_mode': '前进', 'now_distance': (1, 0, 0, 2), 'target_distance': (0, 1, 1, 1), 'now_xy': (1, 11),
         'target_xy': (3, 11)},
        {'move_mode': '左转', 'now_distance': (0, 1, 1, 1), 'target_distance': (1, 0, 1, 1), 'now_xy': (3, 11),
         'target_xy': (3, 11)},
        {'move_mode': '前进', 'now_distance': (1, 0, 1, 1), 'target_distance': (0, 2, 2, 0), 'now_xy': (3, 11),
         'target_xy': (3, 13)},
        {'move_mode': '右转', 'now_distance': (0, 2, 2, 0), 'target_distance': (2, 2, 0, 0), 'now_xy': (3, 13),
         'target_xy': (3, 13)},
        {'move_mode': '前进', 'now_distance': (2, 2, 0, 0), 'target_distance': (1, 1, 1, 0), 'now_xy': (3, 13),
         'target_xy': (5, 13)},
        {'move_mode': '前进', 'now_distance': (1, 1, 1, 0), 'target_distance': (0, 0, 2, 2), 'now_xy': (5, 13),
         'target_xy': (7, 13)},
        {'move_mode': '左转', 'now_distance': (0, 0, 2, 2), 'target_distance': (2, 0, 0, 2), 'now_xy': (7, 13),
         'target_xy': (7, 13)},
        {'move_mode': '前进', 'now_distance': (2, 0, 0, 2), 'target_distance': (1, 1, 1, 0), 'now_xy': (7, 13),
         'target_xy': (7, 15)},
        {'move_mode': '前进', 'now_distance': (1, 1, 1, 0), 'target_distance': (0, 0, 2, 1), 'now_xy': (7, 15),
         'target_xy': (7, 17)},
        {'move_mode': '左转', 'now_distance': (0, 0, 2, 1), 'target_distance': (1, 0, 0, 2), 'now_xy': (7, 17),
         'target_xy': (7, 17)},
        {'move_mode': '前进', 'now_distance': (1, 0, 0, 2), 'target_distance': (0, 1, 1, 0), 'now_xy': (7, 17),
         'target_xy': (5, 17)},
        {'move_mode': '右转', 'now_distance': (0, 1, 1, 0), 'target_distance': (1, 1, 0, 0), 'now_xy': (5, 17),
         'target_xy': (5, 17)},
        {'move_mode': '前进', 'now_distance': (1, 1, 0, 0), 'target_distance': (0, 0, 1, 1), 'now_xy': (5, 17),
         'target_xy': (5, 19)},
        {'move_mode': '左转', 'now_distance': (0, 0, 1, 1), 'target_distance': (1, 0, 0, 1), 'now_xy': (5, 19),
         'target_xy': (5, 19)},
        {'move_mode': '前进', 'now_distance': (1, 0, 0, 1), 'target_distance': (0, 0, 1, 1), 'now_xy': (5, 19),
         'target_xy': (3, 19)},
        {'move_mode': '左转', 'now_distance': (0, 0, 1, 1), 'target_distance': (1, 0, 0, 1), 'now_xy': (3, 19),
         'target_xy': (3, 19)},
        {'move_mode': '前进', 'now_distance': (1, 0, 0, 1), 'target_distance': (0, 1, 1, 0), 'now_xy': (3, 19),
         'target_xy': (3, 17)},
        {'move_mode': '右转', 'now_distance': (0, 1, 1, 0), 'target_distance': (1, 1, 0, 0), 'now_xy': (3, 17),
         'target_xy': (3, 17)},
        {'move_mode': '前进', 'now_distance': (1, 1, 0, 0), 'target_distance': (0, 1, 1, 0), 'now_xy': (3, 17),
         'target_xy': (1, 17)},
        {'move_mode': '右转', 'now_distance': (0, 1, 1, 0), 'target_distance': (1, 1, 0, 0), 'now_xy': (1, 17),
         'target_xy': (1, 17)},
        {'move_mode': '前进', 'now_distance': (1, 1, 0, 0), 'target_distance': (0, 0, 1, 0), 'now_xy': (1, 17),
         'target_xy': (1, 19)}]]

