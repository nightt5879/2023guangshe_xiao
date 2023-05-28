import serial
import RPi.GPIO as GPIO

class Car:
    def __init__(self):
        """
        init the car
        """
        self.car_com1 = serial.Serial("/dev/ttyAMA2", 115200)  # init the car com
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

class Infrared:
    def __init__(self):
        """
        init the infrared
        """
        GPIO.setmode(GPIO.BOARD)
        #the infrareds are from left to right, i.e. left outside left inside right inside right outside
        GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def infrared_read(self,infrared_value):
        """
        the infrared read
        :return the infrared value: it is a list, the first element is the left outside infrared value,
        """
        infrared_value[0] = GPIO.input(27)
        infrared_value[1] = GPIO.input(21)
        infrared_value[2] = GPIO.input(17)
        infrared_value[3] = GPIO.input(22)
        return infrared_value
