# The servo module, which is used to control the servo motor
# the class HalfCircleServo for the 180° servo and class FullCircleServo for the 360° servo

# import the necessary modules
# create three threads to control the servo and read the angle
import threading
import RPi.GPIO as GPIO
import time

# define the servo's GEAR
SLOW = 1
NORMAL = 2
FAST = 3


class Servo:
    """
    The servo class, which is used to control the servo motor

    :param pin: the servo's gpio pin
    """
    def __init__(self, pin):
        self.pwm = None
        self.pin = pin
        self.target = 0

    def set_mode(self):
        GPIO.setup(self.pin, GPIO.OUT, initial=False)
        self.pwm = GPIO.PWM(self.pin, 50)
        self.pwm.start(0)


class HalfCircleServo(Servo):
    """
    The half circle servo class, which is used to control the 180° servo motor, can directly set the angle

    :param pin: the servo's gpio pin
    """
    def __init__(self, pin):
        super().__init__(pin)
        self.angle = 0
        self.set_mode()

    def set_mode(self):
        super().set_mode()
        # initialize the servo's angle
        self.set_angle(self.angle)

    def set_angle(self, angle):
        self.pwm.ChangeDutyCycle(2.5 + 10 * angle / 180)
        time.sleep(0.5)
        self.pwm.ChangeDutyCycle(0)
        # update the servo's angle
        self.angle = angle

    def get_angle(self):
        return self.angle


class FullCircleServo(Servo):
    """
    The full circle servo class, which is used to control the 360° servo motor, can't directly set the angle,
    so we need to read the angle from the file, and set the angle by controlling the servo's runtime

    :param pin: the servo's gpio pin
    :param speed_level: the servo's speed GEAR, is divided into three gears, SLOW, NORMAL, FAST
    """
    def __init__(self, pin, speed_level=NORMAL):
        super().__init__(pin)
        # the matching frequency of the speed GEAR, the unit is circle per second
        self.speed_dict = {SLOW: 1, NORMAL: 2, FAST: 3}
        self.speed = self.get_speed_by_level(speed_level)  # the servo's speed
        self.angle: int = self.read_angle()
        self.set_mode()

    def set_mode(self):
        """
        Set the gpio mode to BCM and set the servo's angle to initial angle
        """
        super().set_mode()
        # initialize the servo's angle
        self.set_angle(self.angle)

    def set_angle(self, angle: int):
        """
        Set the servo's angle by controlling the servo's runtime in the same speed

        :param angle: the servo's angle you want to set
        """
        # calculate the difference between the angle and the servo's angle
        diff = angle - self.angle
        # calculate the servo's runtime, the unit is second
        runtime = (abs(diff) / 360) / self.speed  # 1 circle/second
        # set the pwm for runtime
        self.pwm.ChangeDutyCycle(7.5 + 5 * diff / 360)
        time.sleep(runtime)
        # set the pwm to let the servo stop rotating
        # self.pwm.ChangeDutyCycle(7.5)
        self.pwm.ChangeDutyCycle(0)
        # update the servo's angle
        self.angle = angle
        # write the angle to the file
        with open('servo_angle', 'w') as f:
            f.write(str(self.angle))

    def get_angle(self) -> int:
        return self.angle

    @staticmethod
    def read_angle() -> int:
        with open('servo_angle', 'r') as f:
            angle = int(f.read())
        return angle

    def get_speed_by_level(self, speed_level) -> int:
        """
        Get the servo's speed by the speed GEAR

        :param speed_level: the servo's speed GEAR
        """
        return self.speed_dict[speed_level]


if __name__ == '__main__':
    """
    the process of the create the thread function
    """
    def thread1():
        while True:
            if servo1.target != servo1.angle:
                servo1.set_angle(servo1.target)
            time.sleep(0.5)

    def thread2():
        while True:
            if servo2.target != servo2.angle:
                servo2.set_angle(servo2.target)
            time.sleep(0.5)

    """
    the process of the initialization
    """
    # set the gpio mode to BCM
    GPIO.setmode(GPIO.BCM)

    # define the two servo
    servo1 = FullCircleServo(18, NORMAL)
    servo2 = HalfCircleServo(4)

    # initialize the servo1's angle, because the 360° servo's angle can't read its angle
    servo1.set_angle(0)

    # create two threads to control the servo and read the angle
    t1 = threading.Thread(target=thread1)
    t2 = threading.Thread(target=thread2)
    t1.start()
    t2.start()

    """
    the process of the control other parts and set the servo's angle
    """
    while True:
        # read the input from the keyboard
        cmd = input('>>>')
        start = time.time()
        if cmd == 'q':
            break
        else:
            # set the servo2's angle, the most key code to change the servo's angle
            servo2.target = int(cmd)
        consume = time.time() - start
        print('consume: ', consume)






